"""One step of an anchor survey: the ``UWB_SURVEY`` command, the report an
anchor answers with, and the step as one operation on the extension.

In a ranging round only the responders measure, each its range to the
initiator, so a survey makes every anchor of a group the initiator in turn.
A step is commanded by writing ``UWB_SURVEY`` to every anchor of the group;
an anchor holds it in RAM for the hold time the command carries and returns
to its stored session by itself when no further command comes.

This module is the server's whole use of the firmware SDK's survey support
(``rtlslink.survey`` and the ``survey_report`` protocol event), which the
pinned SDK does not have yet. It follows the firmware's wire contract
(rtls-link-zephyr ``docs/reference/anchor-geometry.md``, "Survey step" and
"Survey report") and reads the report frames off the datagrams itself.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Literal, Optional

import trio
from rtlslink import ProtocolEvent
from rtlslink.dialect import load_dialect

if TYPE_CHECKING:
    from .extension import RtlsExtension

__all__ = (
    "END",
    "MAX_GROUP",
    "REPORT_EVENT",
    "ROUNDS",
    "Member",
    "ReportScanner",
    "StepProblem",
    "StepReport",
    "StepResult",
    "encode_step",
    "end_survey",
    "hold_for",
    "next_step_id",
    "run_step",
    "time_limit",
)

PARAM = "UWB_SURVEY"
#: the command that ends a survey; an anchor accepts it whenever it operates
END = "0000"
MAX_GROUP = 8
ROUNDS = range(20, 1001)

REPORT_EVENT = "survey_report"

ROLE_RESPONDER = 3
STATE_COMPLETE = 2
STATE_FAILED = 3
_ROLES = (2, ROLE_RESPONDER)
_STATES = (1, STATE_COMPLETE, STATE_FAILED)

#: an ack comes within milliseconds, so a missing one is a lost datagram and
#: the write is sent again: four sends in all
ACK_TIMEOUT_S = 0.3
ACK_ATTEMPTS = 4

#: an anchor holds a step this much longer than the server waits for it
HOLD_MARGIN_S = 1.0
#: the longest hold the command carries; a longer step is kept alive by
#: repeating its command, which restarts nothing
MAX_HOLD_S = 10.0

_STEP_IDS = range(1, 0x10000)
_HOLD_DS = range(10, 101)
_MACS = range(1, 0xFFFF)

#: the frames of a report; every report carries the first
_REPORT_MARKER = b"svid"
_REPORT_FIELDS = frozenset(
    ("svid", "svrole", "svst", "svrnd", "svn", "svpeer", "svrng", "svmad", "svok")
)


def encode_step(
    step_id: int,
    rounds: int,
    hold: float,
    initiator_mac: int,
    responder_macs: Sequence[int],
) -> str:
    """The ``UWB_SURVEY`` value of one step. ``hold`` is in seconds, rounded
    up to the tenth the command carries; the responders are in slot order.

    Raises:
        ValueError: for a value the command cannot carry.
    """
    hold_ds = math.ceil(round(hold * 10, 6))
    macs = [initiator_mac, *responder_macs]
    if step_id not in _STEP_IDS:
        raise ValueError(f"step ID must be 1..0xFFFF, not {step_id}")
    if rounds not in ROUNDS:
        raise ValueError(f"rounds must be 20..1000, not {rounds}")
    if hold_ds not in _HOLD_DS:
        raise ValueError(f"hold must be 1..10 s, not {hold:g}")
    if not 2 <= len(macs) <= MAX_GROUP:
        raise ValueError(f"a step has 1..7 responders, not {len(macs) - 1}")
    if any(mac not in _MACS for mac in macs) or len(set(macs)) != len(macs):
        raise ValueError(f"anchor MACs must be distinct and 1..0xFFFE: {macs}")
    head = f"{step_id:04x}{rounds:04x}{hold_ds:02x}"
    return head + "".join(f"{mac:04x}" for mac in macs)


def time_limit(rounds: int, block_ms: int) -> float:
    """How long to wait for a step's reports: the role change, then three
    times the collection."""
    return 1.5 + 3 * rounds * block_ms / 1000


def hold_for(rounds: int, block_ms: int) -> float:
    """The hold to command for a step waited for :func:`time_limit`."""
    return min(MAX_HOLD_S, time_limit(rounds, block_ms) + HOLD_MARGIN_S)


def next_step_id(step_id: int) -> int:
    return step_id % 0xFFFF + 1


@dataclass(frozen=True)
class Member:
    """An anchor of a survey group: where to write, and its ``UWB_MAC``."""

    system_id: int
    mac: int


@dataclass(frozen=True)
class StepReport:
    """What one anchor reports of the step it is in.

    A complete report has ``samples``: a responder's inlier samples, an
    initiator's rounds led. A complete responder also has ``peer_mac`` (the
    step's initiator), ``range_m``, ``mad_m`` and ``valid`` (its valid
    ranges before outlier rejection). With no sample its range is 0, not a
    range.
    """

    system_id: int
    step_id: int
    role: int
    state: int
    samples: Optional[int] = None
    peer_mac: Optional[int] = None
    range_m: Optional[float] = None
    mad_m: Optional[float] = None
    valid: Optional[int] = None

    @classmethod
    def from_event(cls, event: ProtocolEvent) -> StepReport:
        data = event.data
        return cls(
            system_id=event.system_id,
            step_id=data["stepId"],
            role=data["role"],
            state=data["state"],
            samples=data.get("samples"),
            peer_mac=data.get("peerMac"),
            range_m=data.get("rangeM"),
            mad_m=data.get("madM"),
            valid=data.get("valid"),
        )


def _exact(values: dict[str, float], name: str, limit: int) -> Optional[int]:
    """A frame's value as the integer it carries, None when it is absent or
    is not one in 0..limit."""
    value = values.get(name, math.nan)
    if not math.isfinite(value) or value != int(value) or not 0 <= value <= limit:
        return None
    return int(value)


def _report(values: dict[str, float]) -> Optional[dict[str, Any]]:
    """The report the frames of one datagram make, None when a frame its
    role and state require is missing or does not decode."""
    step_id = _exact(values, "svid", 0xFFFF)
    role = _exact(values, "svrole", 0xFF)
    state = _exact(values, "svst", 0xFF)
    if not step_id or role not in _ROLES or state not in _STATES:
        return None
    report: dict[str, Any] = {"stepId": step_id, "role": role, "state": state}
    if state != STATE_COMPLETE:
        return report
    report["samples"] = _exact(values, "svn", 0xFFFF)
    if role != ROLE_RESPONDER:
        return report
    peer, valid = _exact(values, "svpeer", 0xFFFE), _exact(values, "svok", 0xFFFF)
    range_m, mad_m = values.get("svrng", math.nan), values.get("svmad", math.nan)
    if None in (report["samples"], peer, valid) or not (
        math.isfinite(range_m) and math.isfinite(mad_m)
    ):
        return None
    return report | {"peerMac": peer, "rangeM": range_m, "madM": mad_m, "valid": valid}


class ReportScanner:
    """Reads the survey reports off management datagrams, as the events the
    SDK will emit for them. An anchor sends all frames of a report in one
    datagram with one ``time_boot_ms``, so a datagram is scanned on its own.
    """

    def __init__(self) -> None:
        self._parser: Any = None

    def events(self, data: bytes) -> list[ProtocolEvent]:
        from .extension import _named_value_name

        if _REPORT_MARKER not in data:
            return []
        if self._parser is None:
            self._parser = load_dialect().MAVLink(None)
            self._parser.robust_parsing = True
        frames: dict[tuple[int, int], dict[str, float]] = {}
        for message in self._parser.parse_buffer(data) or []:
            if message.get_type() != "NAMED_VALUE_FLOAT":
                continue
            name = _named_value_name(message.name)
            if name in _REPORT_FIELDS:
                key = (message.get_srcSystem(), int(message.time_boot_ms))
                frames.setdefault(key, {})[name] = float(message.value)
        return [
            ProtocolEvent(REPORT_EVENT, system_id, report)
            for (system_id, _), values in frames.items()
            if (report := _report(values)) is not None
        ]


@dataclass(frozen=True)
class StepProblem:
    """Why an anchor has no complete report for a step.

    ``refused``: its ack refused the command. ``no-ack``: it answered no
    write. ``offline``: it is not, or no longer, a live device. With any of
    these no survey can run. ``failed``: its radio did not start the step's
    session. ``no-report``: it sent no complete report in time.
    """

    kind: Literal["refused", "no-ack", "offline", "failed", "no-report"]
    detail: str

    @property
    def ends_survey(self) -> bool:
        return self.kind in ("refused", "no-ack", "offline")


@dataclass(frozen=True)
class StepResult:
    """The complete report of each anchor that sent one, and a problem for
    each anchor that did not, both by system ID."""

    reports: dict[int, StepReport]
    problems: dict[int, StepProblem]

    @property
    def complete(self) -> bool:
        return not self.problems

    @property
    def ends_survey(self) -> bool:
        return any(problem.ends_survey for problem in self.problems.values())


async def _write(
    ext: "RtlsExtension", system_id: int, command: str
) -> Optional[StepProblem]:
    try:
        ack = await ext.set_param(
            system_id,
            PARAM,
            command,
            "custom",
            timeout=ACK_TIMEOUT_S * ACK_ATTEMPTS,
            exact=True,
            resend_after=ACK_TIMEOUT_S,
        )
    except trio.TooSlowError:
        return StepProblem("no-ack", "did not answer")
    except KeyError:
        return StepProblem("offline", "is offline")
    return None if ack["accepted"] else StepProblem("refused", "refused")


async def _write_all(
    ext: "RtlsExtension", group: Sequence[Member], command: str
) -> dict[int, StepProblem]:
    """Writes ``command`` to every anchor of ``group`` at once: a problem
    for each that did not accept it."""
    problems: dict[int, StepProblem] = {}

    async def write(system_id: int) -> None:
        problem = await _write(ext, system_id, command)
        if problem is not None:
            problems[system_id] = problem

    async with trio.open_nursery() as nursery:
        for member in group:
            nursery.start_soon(write, member.system_id)
    return problems


async def run_step(
    ext: "RtlsExtension",
    group: Sequence[Member],
    initiator: Member,
    step_id: int,
    rounds: int,
    hold: float,
    *,
    timeout: float,
) -> StepResult:
    """One step: ``initiator`` leads ``rounds`` rounds with the rest of
    ``group`` as its responders, in the order ``group`` lists them.

    Writes the command to every anchor at once, then waits up to ``timeout``
    seconds for a complete report from each, repeating the command when the
    anchors would not hold the step that long. Returns at once when an anchor
    does not take the command or goes offline. The anchors are left in the
    step: the next step or :func:`end_survey` moves them on.
    """
    responders = [member.mac for member in group if member != initiator]
    command = encode_step(step_id, rounds, hold, initiator.mac, responders)
    waiting = {member.system_id for member in group}
    reports: dict[int, StepReport] = {}
    renew_every = hold / 2 if hold < timeout + HOLD_MARGIN_S else math.inf

    with ext._subscribed_events() as events:
        problems = await _write_all(ext, group, command)
        deadline = trio.current_time() + timeout
        renew_at = trio.current_time() + renew_every
        while waiting - reports.keys() - problems.keys():
            now = trio.current_time()
            if now >= deadline or any(p.ends_survey for p in problems.values()):
                break
            if now >= renew_at:
                renew_at = now + renew_every
                problems.update(await _write_all(ext, group, command))
                continue
            with trio.move_on_at(min(deadline, renew_at)):
                event = await events.receive()
                if event.system_id not in waiting:
                    continue
                if event.kind == "lost":
                    problems[event.system_id] = StepProblem("offline", "went offline")
                elif event.kind == REPORT_EVENT and event.data["stepId"] == step_id:
                    report = StepReport.from_event(event)
                    if report.state == STATE_COMPLETE:
                        reports[report.system_id] = report
                    elif report.state == STATE_FAILED:
                        problems[report.system_id] = StepProblem(
                            "failed", "its radio did not start the step"
                        )

    for system_id in waiting - reports.keys() - problems.keys():
        problems[system_id] = StepProblem("no-report", "sent no complete report")
    return StepResult(reports, problems)


async def end_survey(
    ext: "RtlsExtension", group: Sequence[Member]
) -> dict[int, StepProblem]:
    """Returns every anchor of ``group`` to its stored session: a problem for
    each that did not accept, which returns by itself when its hold is over."""
    return await _write_all(ext, group, END)
