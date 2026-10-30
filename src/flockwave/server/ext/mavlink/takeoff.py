from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from logging import Logger
from time import time
from typing import TYPE_CHECKING

from blinker import Signal
from trio import TASK_STATUS_IGNORED, move_on_after, open_nursery

from flockwave.server.ext.show.takeoff import (
    ScheduledTakeoffManager,
    SimpleScheduledTakeoffManagerBase,
    TakeoffConfiguration,
)
from flockwave.server.ext.signals import SignalsExtensionAPI
from flockwave.server.model.log import Severity

from .channel import Channel
from .packets import create_start_time_configuration_packet

__all__ = ("ScheduledTakeoffManager",)

if TYPE_CHECKING:
    from .driver import MAVLinkUAV
    from .network import MAVLinkNetwork
    from .packets import ShowScheduleStatus
    from .schedule import ShowSchedule
    from .types import MAVLinkMessageSpecification


def create_mavlink_message_spec_from_takeoff_configuration(
    config: TakeoffConfiguration,
) -> MAVLinkMessageSpecification:
    """Creates a MAVLink message specification for the MAVLink message that we
    need to send to all the drones in order to instruct them to do the
    scheduled takeoff with the given configuration.
    """
    return create_start_time_configuration_packet(
        start_time=config.takeoff_time,
        authorization_scope=config.authorization_scope,
        should_update_takeoff_time=config.should_update_takeoff_time,
    )


class MAVLinkScheduledTakeoffManager(ScheduledTakeoffManager["MAVLinkUAV"]):
    """Class that manages the automatic takeoff process on a single MAVLink
    network.
    """

    _network: MAVLinkNetwork
    """The MAVLink network that owns this scheduled takeoff manager."""

    def __init__(self, network: MAVLinkNetwork):
        """Constructor.

        Parameters:
            network: the network whose automatic takeoff process this object
                manages
        """
        super().__init__(log=network.log)
        self._network = network

    async def run(self, *, task_status=TASK_STATUS_IGNORED) -> None:
        async with open_nursery() as nursery:
            nursery.start_soon(self._broadcast_show_schedule)
            await super().run(task_status=task_status)

    @property
    def _schedule(self) -> ShowSchedule:
        return self._network.driver.show_schedule

    async def _broadcast_show_schedule(self) -> None:
        """Broadcasts the show schedule next to the start configuration, more
        often shortly before the start and at once when it changes.
        """
        schedule = self._schedule
        while True:
            changed = schedule.change_event()
            now = time()
            schedule.tick()
            spec = schedule.packet(now)
            if spec is not None:
                try:
                    await self._network.broadcast_packet(
                        spec, channel=Channel.SHOW_CONTROL
                    )
                except Exception:
                    # The drones that missed it get it individually
                    pass
            with move_on_after(schedule.period(now)):
                await changed.wait()

    async def broadcast_takeoff_configuration(
        self, config: TakeoffConfiguration
    ) -> None:
        try:
            spec = create_mavlink_message_spec_from_takeoff_configuration(config)
        except ValueError as ex:
            if self._log:
                self._log.warning(f"Could not create start time config packet: {ex}")
            raise

        await self._network.broadcast_packet(spec, channel=Channel.SHOW_CONTROL)

    def iter_uavs_to_schedule(self) -> Iterator[MAVLinkUAV]:
        """Returns an iterator over the UAVs managed by this object that are
        to be updated on an individual basis if they do not receive the
        broadcast configuration packet or do not respond to it.

        May return an empty iterator if you do not want to support individual
        configuration for the UAVs.
        """
        return (
            uav
            for uav in self._network.uavs()
            if uav.is_connected and uav.supports_scheduled_takeoff
        )

    def check_start_agreement(self, config: TakeoffConfiguration, lead: float) -> None:
        schedule = self._schedule
        now = time()
        for uav in self.iter_uavs_to_schedule():
            echo = uav.show_schedule_status
            if echo is None:
                problem = (
                    "its start time or authorization differs from the server's"
                    if _start_differs(uav, config)
                    else None
                )
            elif (
                config.authorization_scope != uav.scheduled_takeoff_authorization_scope
            ):
                problem = "its authorization differs from the server's"
            elif lead > DRONE_COMMIT_LEAD:
                problem = _problem_before_commit(echo, schedule, now)
            else:
                problem = _problem_after_commit(echo, schedule)
                if echo.has_start and self._log:
                    offset = (echo.start_cluster_us - schedule.start_cluster_us) / 1000
                    self._log.info(
                        f"Start offset at T-{lead:.0f} s: {offset:+.2f} ms",
                        extra={"id": uav.id},
                    )
            if problem is None:
                continue
            message = f"Not in sync for the scheduled start: {problem}"
            if self._log:
                self._log.warning(message, extra={"id": uav.id})
            uav.send_log_message_to_gcs(message, severity=Severity.WARNING)

    def uav_needs_update(self, uav: MAVLinkUAV, config: TakeoffConfiguration) -> bool:
        """Returns whether the given UAV needs to be updated if the desired
        takeoff configuration is the one provided as `config`.

        May return False unconditionally if you do not want to support individual
        configuration for the UAVs.

        Args:
            uav: the UAV to check
            config: the desired takeoff configuration to check against
        """
        if config.authorization_scope != uav.scheduled_takeoff_authorization_scope:
            # Auth scope is different so we definitely need an update
            return True

        echo = uav.show_schedule_status
        if echo is not None:
            return self._schedule.packet(time()) is not None and (
                self._schedule.disagrees_with(echo)
            )

        if config.should_update_takeoff_time:
            # Takeoff time must be cleared (None) or set to a specific
            # value; we need an update if it is different from what
            # we have on the UAV
            return (
                uav.scheduled_takeoff_time != config.takeoff_time_in_whole_seconds
                or uav.scheduled_takeoff_time_msec != config.takeoff_time_msec
            )

        # Auth scope is the same and the takeoff time does not
        # need to change
        return False

    async def update_uav(self, uav: MAVLinkUAV, config: TakeoffConfiguration) -> None:
        desired_auth_scope = config.authorization_scope
        desired_takeoff_time = config.takeoff_time_in_legacy_format

        echo = uav.show_schedule_status
        desired_msec = config.takeoff_time_msec
        if echo is not None:
            spec = self._schedule.packet(time())
            if spec is not None and self._schedule.disagrees_with(echo):
                await self._network.send_packet(spec, uav, channel=Channel.SHOW_CONTROL)
        elif (desired_takeoff_time is None or desired_takeoff_time >= 0) and (
            desired_takeoff_time != uav.scheduled_takeoff_time
            or desired_msec != uav.scheduled_takeoff_time_msec
        ):
            await uav.set_scheduled_takeoff_time(
                seconds=desired_takeoff_time, msec=desired_msec
            )

        if desired_auth_scope != uav.scheduled_takeoff_authorization_scope:
            await uav.set_authorization_scope(desired_auth_scope)


class ScheduledTakeoffSignalDispatcher(SimpleScheduledTakeoffManagerBase):
    """Class that dispatches a signal via the signals API whenever a scheduled takeoff
    configuration is acted upon by the MAVLink networks.
    """

    _signal: Signal | None = None
    """The signal to dispatch when a scheduled takeoff configuration is acted upon by
    the MAVLink networks. `None` to do nothing.

    The signal must take a single keyword argument named `spec` that contains the
    MAVLink message specification for the packet that we need to send to all the drones
    in order to update the scheduled takeoff time and the authorization state.
    """

    async def broadcast_takeoff_configuration(
        self, config: TakeoffConfiguration
    ) -> None:
        if self._signal:
            spec = create_mavlink_message_spec_from_takeoff_configuration(config)
            self._signal.send(self, spec=spec)

    @contextmanager
    def use(
        self, signals: SignalsExtensionAPI, *, log: Logger | None = None
    ) -> Iterator[None]:
        """Context manager that sets up the signal to dispatch and the logger to use for
        the duration of the context.
        """
        old_signal = self._signal
        self._signal = signals.get("mavlink:show_control")

        old_log = self._log
        self._log = log or old_log

        try:
            yield
        finally:
            self._signal = old_signal
            self._log = old_log


START_AGREEMENT_TOLERANCE = 0.02
"""Seconds by which a drone's reported start may differ from the server's in
the start agreement check: the start may move by a few milliseconds shortly
before it (e.g. when the timecode clock is re-anchored), and the drones follow
it at the next broadcast."""


def _start_differs(uav: MAVLinkUAV, config: TakeoffConfiguration) -> bool:
    if config.authorization_scope != uav.scheduled_takeoff_authorization_scope:
        return True
    if not config.should_update_takeoff_time:
        return False
    desired, reported = config.takeoff_time, uav.scheduled_takeoff_time
    if desired is None or reported is None:
        return desired != reported
    reported_time = reported + uav.scheduled_takeoff_time_msec / 1000
    return abs(reported_time - desired) > START_AGREEMENT_TOLERANCE


DRONE_COMMIT_LEAD = 10
"""Number of seconds before the start when the drones commit to it."""

ECHO_MAX_AGE = 3.0
"""Seconds after which a drone's show schedule echo is out of date."""

START_TOLERANCE_BEFORE_COMMIT_US = 1000
"""Microseconds by which a drone's echoed start may differ from the sent one
before the drones commit: one re-sent start may be on its way."""

START_TOLERANCE_AFTER_COMMIT_US = 5000
"""Microseconds by which a committed drone's start, which moves with the drift
of its crystal, may differ from the frozen start."""


def _problem_before_commit(
    echo: ShowScheduleStatus, schedule: ShowSchedule, now: float
) -> str | None:
    if now - echo.received_at > ECHO_MAX_AGE:
        return f"its show schedule echo is {now - echo.received_at:.0f} s old"
    if not echo.clock_valid:
        return "it has no valid show clock"
    if echo.schedule_id != schedule.id:
        return f"it holds schedule {echo.schedule_id}, not {schedule.id}"
    if not echo.has_start:
        return "it holds no start"
    return _start_off(echo, schedule, START_TOLERANCE_BEFORE_COMMIT_US)


def _problem_after_commit(
    echo: ShowScheduleStatus, schedule: ShowSchedule
) -> str | None:
    if not echo.committed:
        return "it has not committed to the start"
    if echo.schedule_id != schedule.id:
        return f"it holds schedule {echo.schedule_id}, not {schedule.id}"
    return _start_off(echo, schedule, START_TOLERANCE_AFTER_COMMIT_US)


def _start_off(
    echo: ShowScheduleStatus, schedule: ShowSchedule, tolerance_us: int
) -> str | None:
    off = echo.start_cluster_us - schedule.start_cluster_us
    if abs(off) > tolerance_us:
        return f"its start is {off / 1000:+.1f} ms off the server's"
    return None
