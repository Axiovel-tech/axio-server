"""The anchor survey as the operator runs it (X-RTLS-SURVEY).

A survey ranges every pair of a group of anchors (``survey_step.py``), fits
their positions in the cell frame (``survey_fit.py``) and, when the operator
commits, writes every anchor its own position (``survey_commit.py``). One
survey at a time::

    idle -> measuring -> returning -> ready -> committing -> committed

A run that fails or is cancelled still returns the anchors to their stored
session, then ends in ``failed`` or ``idle``, and so does a measurement the
fit cannot start from; a commit an anchor misses falls back to ``ready``. The measurement and the fit are kept in memory for the
page only: after a restart the positions come from the anchors. A server
that dies mid-run leaves anchors that return by themselves when the hold
time of their last step is over. The lock's resurvey check (``check``)
measures in the same way, alongside what the survey holds, and keeps
nothing.

The message, its state object and the refusals: AxioStack
``contracts/rtls-client-api.md``, ``X-RTLS-SURVEY``.
"""

from __future__ import annotations

import copy
import random
import time
from collections.abc import Mapping
from dataclasses import dataclass, field, replace
from typing import TYPE_CHECKING, Any, Optional

import trio
from rtlslink.survey import ROLE_RESPONDER, SurveyAnchor, hold_for, time_limit

from .anchor_lock import anchor_views
from .cell_settings import write_refusal
from .lock_resurvey import stored_frame
from .survey_commit import (
    CONFIRM_TIMEOUT,
    CommitRefused,
    Plan,
    commit_refusal,
    plan_commit,
    read_stored,
    running_ids,
    stored_positions,
    write_locations,
)
from .survey_fit import (
    DirectedRange,
    Frame,
    Point,
    SurveyFit,
    SurveyFitError,
    fit_survey,
    frame_refusal,
)
from .survey_group import Anchor, armed_refusal, block_ms, select_group, step_order
from .survey_notify import Pulse, StatePusher
from .survey_request import (
    DEFAULT_ROUNDS,
    Placement,
    frame_json,
    parse_frame,
    parse_placement,
    parse_start,
    placement_json,
)
from .survey_step import StepResult, end_survey, next_step_id, run_step

if TYPE_CHECKING:
    from .extension import RtlsExtension

__all__ = ("Check", "Survey", "handle_survey_request")

RUNNING_STATES = ("measuring", "returning", "committing")

SURVEY_RUNNING = "A survey is running"
CHECK_RUNNING = "A resurvey check is running"

INCOMPLETE = "Not every anchor took the survey"
#: why a commit or a fit is refused that does not name the held survey's
#: revision: another client changed the survey since the request's was shown
STALE = "stale"
CANCELLED = "cancelled"
_INTERNAL_ERROR = "The survey stopped on an internal error"

#: what an anchor reports of the session it runs: -1 (here None) while none
_SESSION_STATS = ("cellConfig", "txGain", "surveyId")

#: an uptime that falls by more than this comes from an anchor that booted
#: again, not from frames that swapped places, in ms
_REORDER_MS = 5000
#: the uptime has 32 bits: a fall that is a step forward of at most this
#: across their wrap (49.7 days) is the wrap, in ms
_WRAP_STEP_MS = 60_000

_Answer = tuple[str, Optional[str]]


@dataclass
class _Run:
    """One survey: its group, what it measured and what was made of it."""

    #: the group as the request listed it, and in the order of its steps
    anchors: list[Anchor]
    order: list[Anchor]
    rounds: int
    block_ms: int
    frame: Frame
    placement: Optional[Placement]
    #: each anchor's boot count and the uptime of its stat frames, as last
    #: seen; None while not reported
    booted: dict[int, tuple[Optional[int], Optional[int]]]
    #: by ``(initiator, responder)`` system ID
    measurements: dict[tuple[int, int], DirectedRange] = field(default_factory=dict)
    step: int = 1
    fit: Optional[SurveyFit] = None
    #: the ``commit`` member of the state
    commit: Optional[dict[str, Any]] = None
    #: what the commit of this fit wrote, while an anchor has yet to take it
    plan: Optional[Plan] = None
    #: since the run started, the server lost an anchor
    lost: bool = False
    #: the anchors that booted again since the run started: such an anchor
    #: may have been carried away while it was off
    rebooted: set[int] = field(default_factory=set)
    cancelled: bool = False
    scope: trio.CancelScope = field(default_factory=trio.CancelScope)
    #: the anchors that reported ranging since the end command was answered
    returned: set[int] = field(default_factory=set)

    @property
    def ids(self) -> list[int]:
        return [anchor.system_id for anchor in self.anchors]


@dataclass(frozen=True)
class Check:
    """What the lock's resurvey check measured: the fit, or why there is
    none; the anchors it measured, those of them that restarted while it
    ran, and the positions they store."""

    fit: Optional[SurveyFit]
    reason: Optional[str]
    ids: tuple[int, ...] = ()
    rebooted: frozenset[int] = frozenset()
    stored: Mapping[int, Point] = field(default_factory=dict)


def _restarted(before: int, now: int) -> bool:
    """Whether an uptime that went from ``before`` to ``now`` started again."""
    return before - now > _REORDER_MS and (now - before) % 2**32 > _WRAP_STEP_MS


def _merged(first: StepResult, again: StepResult) -> StepResult:
    """What two attempts at a step measured, neither of which ended the
    survey: the reports of both, and a problem for each anchor that
    completed neither."""
    reports = first.reports | again.reports
    missing = {
        system_id: problem
        for system_id, problem in again.problems.items()
        if system_id not in reports
    }
    return StepResult(reports, missing)


class Survey:
    """The extension's one survey: its state, the run and the commit."""

    def __init__(self, ext: "RtlsExtension", revision: Optional[int] = None):
        self._ext = ext
        self.state = "idle"
        self.reason: Optional[str] = None
        #: raised on every change of what the survey holds and never lowered.
        #: It starts at the wall clock in milliseconds, above what an earlier
        #: instance of the extension sent: the changes are fewer than that.
        self.revision = time.time_ns() // 1_000_000 if revision is None else revision
        self._run: Optional[_Run] = None
        #: the measurement of the lock's resurvey check while it runs: the
        #: survey holds nothing of it
        self._check: Optional[_Run] = None
        self._changed = Pulse()
        self._push = StatePusher(ext, self._notification).notify

    def _notify(self) -> None:
        """Every change of the survey makes a new revision, which is pushed."""
        self.revision += 1
        self._push()

    def _is_current(self, revision: Any) -> bool:
        return type(revision) in (int, float) and revision == self.revision

    @property
    def running(self) -> bool:
        """Whether a run or a resurvey check measures, returns or commits."""
        return self.busy is not None

    @property
    def busy(self) -> Optional[str]:
        """Why nothing else may use the anchors now: a resurvey check or a
        run measures, returns or commits."""
        if self._check is not None:
            return CHECK_RUNNING
        return SURVEY_RUNNING if self.state in RUNNING_STATES else None

    def _active(self) -> list[_Run]:
        """The runs whose anchors the survey has in hand."""
        held = self._run if self.state in RUNNING_STATES else None
        return [run for run in (held, self._check) if run is not None]

    def _watched(self) -> list[_Run]:
        """The runs that follow their anchors' boots and losses: the held
        one, whose fit may yet be committed, and a resurvey check."""
        return [run for run in (self._run, self._check) if run is not None]

    def involves(self, system_id: int) -> bool:
        """Whether a running survey or resurvey check has this anchor in its
        group."""
        return any(system_id in run.ids for run in self._active())

    @property
    def json(self) -> dict[str, Any]:
        """The state object."""
        state: dict[str, Any] = {
            "revision": self.revision,
            "state": self.state,
            "reason": self.reason,
            "anchors": [],
            "step": 0,
            "steps": 0,
            "initiator": None,
            "pairs": [],
            "fit": None,
            "gate": {"blocked": [], "warnings": []},
            "commit": None,
            "frame": None,
            "placement": None,
        }
        run = self._run
        if run is None:
            return state
        state |= {
            "anchors": run.ids,
            "step": run.step,
            "steps": len(run.order),
            "commit": copy.deepcopy(run.commit),
            "frame": frame_json(run.frame),
            "placement": placement_json(run.placement),
        }
        if self.state == "measuring":
            state["initiator"] = run.order[run.step - 1].system_id
        if run.fit is not None:
            state |= run.fit.json
        return state

    def _notification(self) -> dict[str, Any]:
        return {"type": "X-RTLS-SURVEY", "survey": self.json}

    # ---- what the extension tells the survey ----

    def keep_session(
        self, system_id: int, stats: dict[str, Any], previous: dict[str, Any]
    ) -> None:
        """A step, the end of a survey and a commit each restart an anchor's
        ranging, and for about 0.3 s it reports no session. While the survey
        runs, its anchors keep the session they reported last in ``stats``,
        so that a restart reads neither as cell settings that differ nor as
        an anchor without a survey."""
        if self.involves(system_id):
            for key in _SESSION_STATS:
                if stats.get(key) is None and previous.get(key) is not None:
                    stats[key] = previous[key]

    def on_stats(self, system_id: int, stats: dict[str, Any]) -> None:
        for run in self._watched():
            self._follow(run, system_id, stats)
        self._changed.set()

    def _follow(self, run: _Run, system_id: int, stats: dict[str, Any]) -> None:
        if system_id in run.booted:
            if (stats.get("rangingRateHz") or 0) > 0:
                run.returned.add(system_id)
            boots, uptime = run.booted[system_id]
            now_boots, now_uptime = self._boot_marks(system_id, stats)
            if (None not in (boots, now_boots) and now_boots != boots) or (
                uptime is not None
                and now_uptime is not None
                and _restarted(uptime, now_uptime)
            ):
                run.rebooted.add(system_id)
            run.booted[system_id] = (
                boots if now_boots is None else now_boots,
                uptime if now_uptime is None else now_uptime,
            )

    def _boot_marks(
        self, system_id: int, stats: dict[str, Any]
    ) -> tuple[Optional[int], Optional[int]]:
        """What tells that an anchor booted again: its boot count, which
        comes with the status report, and the uptime of its last stat
        frame, which starts again from zero."""
        device = self._ext._require_protocol().devices.get(system_id)
        return stats.get("bootCount"), getattr(device, "stats_time_boot_ms", None)

    def on_lost(self, system_id: int) -> None:
        for run in self._watched():
            if system_id in run.booted:
                run.lost = True

    # ---- the actions ----

    async def status(self, body: dict[str, Any]) -> _Answer:
        return "status", None

    async def start(self, body: dict[str, Any]) -> _Answer:
        requested, rounds, raw_frame = parse_start(body)
        placement = parse_placement(body.get("placement"))
        ext = self._ext
        nursery = ext._nursery
        anchors, reason = self._group(requested)
        if reason is not None or nursery is None:
            return "refused", reason or "RTLS is not running"
        ids = [anchor.system_id for anchor in anchors]
        try:
            frame = parse_frame(raw_frame, ids)
        except SurveyFitError as ex:
            return "refused", str(ex)
        reason = frame_refusal(frame, ids)
        if reason is not None:
            return "refused", reason
        run = self._run = self._new_run(anchors, rounds, frame, placement)
        self.state, self.reason = "measuring", None
        nursery.start_soon(self._survey, run)
        self._notify()
        return "started", None

    def _group(
        self, requested: Optional[list[int]]
    ) -> tuple[list[Anchor], Optional[str]]:
        """The group of a run, or why none may start now."""
        ext = self._ext
        if self.busy is not None:
            return [], self.busy
        if ext._cell_applying:
            return [], "An apply is running"
        if ext._nursery is None:
            return [], "RTLS is not running"
        reason = write_refusal(ext) or armed_refusal(ext)
        if reason is not None:
            return [], reason
        anchors, reason = select_group(ext, requested)
        updating = [a.name for a in anchors if ext.ota_running(a.system_id)]
        if updating:
            return [], f"{updating[0]} is being updated"
        return anchors, reason

    def _new_run(
        self,
        anchors: list[Anchor],
        rounds: int,
        frame: Frame,
        placement: Optional[Placement],
    ) -> _Run:
        ext = self._ext
        return _Run(
            anchors=anchors,
            order=step_order(ext, anchors),
            rounds=rounds,
            block_ms=block_ms(ext, anchors),
            frame=frame,
            placement=placement,
            booted={
                anchor.system_id: self._boot_marks(
                    anchor.system_id, ext._fresh_stats(anchor.system_id) or {}
                )
                for anchor in anchors
            },
        )

    async def check(self) -> Check:
        """The lock's resurvey check: measures the anchors a start without a
        group takes, as a start does, and fits them in the frame of the
        positions they store. The held run, its fit and the revision stay as
        they were, and nothing is written. Refused, and refusing a start and
        a commit, as a running survey."""
        ext = self._ext
        if self.busy is not None:
            return Check(None, self.busy)
        online = [view["systemId"] for view in anchor_views(ext) if view["online"]]
        unread = await read_stored(ext, online, fresh=False)
        anchors, reason = self._group(None)
        if reason is not None:
            return Check(None, reason)
        missing = [anchor.name for anchor in anchors if anchor.system_id in unread]
        if missing:
            return Check(None, f"{missing[0]}: its stored survey could not be read")
        ids = [anchor.system_id for anchor in anchors]
        stored = stored_positions(ext, ids)
        frame, reason = stored_frame(stored)
        if frame is None:
            return Check(None, reason)
        run = self._check = self._new_run(anchors, DEFAULT_ROUNDS, frame, None)
        failure: Optional[str] = None
        try:
            try:
                failure = await self._measure(run)
            except Exception as ex:
                failure = self._defect(ex)
        finally:
            # cancelled or not, the anchors go back to ranging
            with trio.CancelScope(shield=True):
                try:
                    await self._return(run)
                except Exception as ex:
                    failure = failure or self._defect(ex)
            self._check = None
        done = Check(None, failure, tuple(ids), frozenset(run.rebooted), stored)
        if failure is not None:
            return done
        try:
            return replace(done, fit=self._fitted(run, frame, None))
        except SurveyFitError as ex:
            return replace(done, reason=str(ex))

    async def cancel(self, body: dict[str, Any]) -> _Answer:
        run = self._run
        if self.state == "committing":
            return "refused", "A commit cannot be cancelled"
        if run is None or self.state not in RUNNING_STATES:
            return "refused", "No survey is running"
        run.cancelled = True
        run.scope.cancel()
        self.state = "returning"
        self._notify()
        return "cancelled", None

    async def fit(self, body: dict[str, Any]) -> _Answer:
        run = self._run
        if "revision" in body and not self._is_current(body["revision"]):
            return "refused", STALE
        if self.busy is not None:
            return "refused", self.busy
        if run is None or self.state != "ready":
            return "refused", "There is no measurement to fit"
        placement = parse_placement(body.get("placement"))
        try:
            frame = run.frame
            if body.get("frame") is not None:
                frame = parse_frame(body["frame"], run.ids)
            fit = self._fitted(run, frame, placement)
        except SurveyFitError as ex:
            return "refused", str(ex)
        run.frame, run.placement, run.fit = frame, placement, fit
        run.commit = run.plan = self.reason = None
        self._notify()
        return "fitted", None

    async def commit(self, body: dict[str, Any]) -> _Answer:
        run = self._run
        if not self._is_current(body.get("revision")):
            return "refused", STALE
        if self.busy is not None:
            return "refused", self.busy
        if run is None or self.state != "ready" or run.fit is None:
            return "refused", "There is no fit to commit"
        fit = run.fit
        reason = self._commit_refusal(run, fit)
        if reason is not None:
            return "refused", reason
        if fit.blocked:
            return "blocked", None
        if fit.warnings and body.get("acknowledgeWarnings") is not True:
            return "warnings", None
        held, self.state, self.reason = self.reason, "committing", None
        self._notify()
        committed = False
        try:
            try:
                plan = await plan_commit(
                    self._ext, run.anchors, fit, run.measurements, run.plan
                )
                # the reads took time: what allowed the commit may be gone
                reason = self._commit_refusal(run, fit)
            except CommitRefused as ex:
                reason = str(ex)
            else:
                if reason is None:
                    run.plan = plan
                    committed = await self._write_and_confirm(run, plan)
        finally:
            self.state = "committed" if committed else "ready"
            self.reason = None if committed else held if reason else INCOMPLETE
        self._notify()
        if reason is not None:
            return "refused", reason
        return ("committed", None) if committed else ("incomplete", INCOMPLETE)

    def _commit_refusal(self, run: _Run, fit: SurveyFit) -> Optional[str]:
        return commit_refusal(
            self._ext,
            run.anchors,
            fit,
            lost=run.lost,
            rebooted=[a.name for a in run.anchors if a.system_id in run.rebooted],
            retry=run.plan is not None,
        )

    async def _write_and_confirm(self, run: _Run, plan: Plan) -> bool:
        """Writes every anchor of the group its position and waits until
        each reports the new survey ID; whether every anchor did both. The
        ID is the one an anchor reports, never the -1 of a restart, which a
        simulated anchor does not show."""
        ext, survey_id = self._ext, plan.survey_id
        running: list[int] = []
        commit = run.commit = {"surveyId": survey_id, "results": {}, "running": running}
        self._notify()
        results = commit["results"] = await write_locations(
            ext, run.anchors, plan.values
        )
        self._notify()
        written = {
            anchor.system_id
            for anchor in run.anchors
            if results[str(anchor.system_id)]["written"] == "changed"
        }
        with trio.move_on_after(CONFIRM_TIMEOUT):
            while True:
                reporting = running_ids(ext, run.anchors, survey_id)
                if reporting != running:
                    running = commit["running"] = reporting
                    self._notify()
                if written <= set(running):
                    break
                await self._changed.wait()
        return written == set(run.ids) == set(running)

    # ---- the run ----

    async def _survey(self, run: _Run) -> None:
        """The run's task: the steps, the return and the fit. Nothing it
        meets may end the extension's nursery, so a defect ends the survey
        in ``failed`` instead."""
        failure: Optional[str] = None
        try:
            with run.scope:
                failure = await self._measure(run)
        except Exception as ex:
            failure = self._defect(ex)
        try:
            self.state = "returning"
            self._notify()
            await self._return(run)
            if failure is None and not run.cancelled:
                await read_stored(self._ext, run.ids, fresh=False)
                try:
                    run.fit = self._fitted(run, run.frame, run.placement)
                except SurveyFitError as ex:
                    failure = str(ex)
        except Exception as ex:
            failure = self._defect(ex)
        if run.cancelled:
            self._run, self.state, self.reason = None, "idle", CANCELLED
        elif failure is not None:
            self.state, self.reason = "failed", failure
        else:
            self.state = "ready"
        self._notify()

    def _defect(self, error: Exception) -> str:
        if self._ext.log:
            self._ext.log.warning(f"rtls: the survey stopped: {error!r}")
        return _INTERNAL_ERROR

    async def _measure(self, run: _Run) -> Optional[str]:
        """Every anchor initiates one step; returns why the run cannot go
        on, if it cannot. A step that ends without every report is run once
        more, then what is missing stays missing: the gate decides."""
        group = [SurveyAnchor(anchor.system_id, anchor.mac) for anchor in run.order]
        limit = time_limit(run.rounds, run.block_ms)
        hold = hold_for(run.rounds, run.block_ms)
        step_id = random.randint(1, 0xFFFF)

        async def step(initiator: SurveyAnchor) -> StepResult:
            nonlocal step_id
            current, step_id = step_id, next_step_id(step_id)
            return await run_step(
                self._ext, group, initiator, current, run.rounds, hold, timeout=limit
            )

        for index, initiator in enumerate(group, start=1):
            if index > 1:
                run.step = index
                if run is self._run:
                    self._notify()
            result = await step(initiator)
            if not (result.complete or result.ends_survey):
                again = await step(initiator)
                result = again if again.ends_survey else _merged(result, again)
            if result.ends_survey:
                return self._step_failure(run, result)
            for system_id, report in result.reports.items():
                if report.role == ROLE_RESPONDER and report.samples:
                    run.measurements[initiator.system_id, system_id] = DirectedRange(
                        range_m=report.range_m or 0.0,
                        mad_m=report.mad_m or 0.0,
                        inliers=report.samples or 0,
                        share=min(1.0, (report.valid or 0) / run.rounds),
                    )
        return None

    def _step_failure(self, run: _Run, result: StepResult) -> str:
        """Names the first anchor that ended the run, and why."""
        system_id = min(s for s, p in result.problems.items() if p.ends_survey)
        problem = result.problems[system_id]
        name = next(a.name for a in run.anchors if a.system_id == system_id)
        if problem.kind != "refused":
            return f"{name} {problem.detail}"
        return f"{name} refused the survey step: {self._why(system_id)}"

    def _why(self, system_id: int) -> str:
        """The ack of a refused step carries no reason: the anchor's stats
        name the likely one."""
        stats = self._ext._fresh_stats(system_id) or {}
        if stats.get("locked"):
            return "locked"
        if stats.get("powerState") not in (None, "operating"):
            return "not operating"
        return "its radio is not running, or the step does not fit its slots"

    async def _return(self, run: _Run) -> None:
        """Ends the survey on every anchor and waits until every boot
        responder reports ranging again. An anchor that did not take the end
        returns by itself, so the wait is then the hold time."""
        group = [SurveyAnchor(anchor.system_id, anchor.mac) for anchor in run.order]
        waiting = {a.system_id for a in run.order if a.boot_responder}
        deadline = trio.current_time() + hold_for(run.rounds, run.block_ms)
        unanswered = await end_survey(self._ext, group)
        run.returned.clear()
        with trio.move_on_at(deadline):
            while unanswered or not waiting <= run.returned:
                await self._changed.wait()

    def _fitted(
        self, run: _Run, frame: Frame, placement: Optional[Placement]
    ) -> SurveyFit:
        return fit_survey(
            run.ids,
            run.measurements,
            frame,
            placement=placement,
            stored=stored_positions(self._ext, run.ids),
        )


_ACTIONS = ("status", "start", "cancel", "fit", "commit")


async def handle_survey_request(
    ext: "RtlsExtension", body: dict[str, Any]
) -> dict[str, Any]:
    """The X-RTLS-SURVEY response body: the action's result, its reason and
    the state object after it.

    Raises:
        ValueError: for an unknown action or a malformed member.
    """
    action = body.get("action", "status")
    if action not in _ACTIONS:
        raise ValueError(
            f"Invalid action: {action!r} (expected one of {', '.join(_ACTIONS)})"
        )
    survey = ext._survey
    result, reason = await getattr(survey, action)(body)
    return {
        "type": "X-RTLS-SURVEY",
        "action": action,
        "result": result,
        "reason": reason,
        "survey": survey.json,
    }
