"""One step of an anchor survey on the extension's own transport.

In a ranging round only the responders measure, each its range to the
initiator, so a survey makes every anchor of a group the initiator in turn.
A step is commanded by writing ``UWB_SURVEY`` to every anchor of the group;
an anchor holds it in RAM for the hold time the command carries and returns
to its stored session by itself when no further command comes.

The command, the report and their timing are the SDK's (``rtlslink.survey``;
firmware: rtls-link-zephyr ``docs/reference/anchor-geometry.md``, "Survey
step" and "Survey report"). Its ``run_step`` needs a client that owns its
socket, which the server does not have, so the write and the wait are done
here over :meth:`RtlsExtension.set_param` and the extension's event stream.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import TYPE_CHECKING, Literal, Optional

import trio
from rtlslink.survey import (
    ACK_ATTEMPTS,
    ACK_TIMEOUT_S,
    END,
    HOLD_MARGIN_S,
    PARAM,
    STATE_COMPLETE,
    STATE_FAILED,
    StepReport,
    SurveyAnchor,
    encode_step,
)

if TYPE_CHECKING:
    from .extension import RtlsExtension

__all__ = (
    "StepProblem",
    "StepResult",
    "end_survey",
    "next_step_id",
    "run_step",
)

REPORT_EVENT = "survey_report"


def next_step_id(step_id: int) -> int:
    return step_id % 0xFFFF + 1


@dataclass(frozen=True)
class StepProblem:
    """Why an anchor has no complete report for a step.

    ``refused``: its ack refused the command. ``no-ack``: it answered no
    write. ``offline``: it is not, or no longer, a live device. With any of
    these no survey can run. ``failed``: its radio did not start the step's
    session, or lost it before the report was complete. ``no-report``: it sent no complete report in time.
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
    ext: "RtlsExtension", group: Sequence[SurveyAnchor], command: str
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
    group: Sequence[SurveyAnchor],
    initiator: SurveyAnchor,
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
                    reported = report.system_id in reports
                    if report.state == STATE_COMPLETE:
                        reports[report.system_id] = report
                    elif report.state == STATE_FAILED and not reported:
                        problems[report.system_id] = StepProblem(
                            "failed", "its radio did not hold the step"
                        )

    for system_id in waiting - reports.keys() - problems.keys():
        problems[system_id] = StepProblem("no-report", "sent no complete report")
    return StepResult(reports, problems)


async def end_survey(
    ext: "RtlsExtension", group: Sequence[SurveyAnchor]
) -> dict[int, StepProblem]:
    """Returns every anchor of ``group`` to its stored session: a problem for
    each that did not accept, which returns by itself when its hold is over."""
    return await _write_all(ext, group, END)
