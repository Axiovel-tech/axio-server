"""Clock that can be used to determine how much time is left until the start
of the show, or the elapsed time into the show if it is already running.
"""

from collections.abc import Callable, Iterator
from contextlib import contextmanager
from logging import Logger
from time import time
from typing import Any

from flockwave.server.model.clock import (
    Clock,
    TimeElapsedSinceReferenceClock,
)

__all__ = ("ShowClock", "ShowEndClock")

#: Adjustments of the primary clock smaller than this, in seconds, are
#: ignored silently while freewheeling (drift corrections, not jumps).
REPORTED_ADJUSTMENT = 0.1

#: A start that becomes known through the primary clock (e.g. timecode that
#: starts rolling) less than this many seconds before it is reported: the
#: drones schedule a start that reaches them after it for the next week, and
#: they prepare for takeoff ten seconds before it.
MIN_START_LEAD = 15.0


class ShowClock(TimeElapsedSinceReferenceClock):
    """Clock that shows the number of seconds elapsed since the scheduled start
    of the drone show.
    """

    def __init__(self):
        """Constructor."""
        super().__init__(id="show", epoch=None)

    @property
    def start_time(self) -> float | None:
        """The scheduled start time, in seconds since the UNIX epoch, or ``None``
        if no start time has been scheduled yet.

        Same as the reference time; kept for compatibility purposes only.
        """
        return self.reference_time

    @start_time.setter
    def start_time(self, value: float | None):
        self.reference_time = value


class ShowEndClock(TimeElapsedSinceReferenceClock):
    """Clock that shows the number of seconds elapsed since the scheduled end
    of the drone show.
    """

    def __init__(self):
        """Constructor."""
        super().__init__(id="end_of_show", epoch=None)


class ClockSynchronizationHandler:
    """Class that holds references to a primary and a secondary clock and that
    synchronizes the two clocks such that the secondary clock is running if and
    only if the primary clock is running and the offset between the two clocks
    is constant.

    The clock synchronization handler has an optional concept called "the point
    of no return". This is a threshold on the number of seconds on the secondary
    clock. Once the running secondary clock has reached it, the secondary clock
    freewheels on the server clock until the end of the show: the primary
    clock stopping, losing its signal, jumping or restarting no longer stops or
    moves it. Synchronizing again to the same primary clock and start keeps it
    freewheeling; only a different start or clock, or stopping the
    synchronization, changes it.
    """

    _enabled: bool = False

    _primary_clock: Clock | None = None
    _secondary_clock: TimeElapsedSinceReferenceClock | None = None

    _subscribed_clock: Clock | None = None
    """The clock whose signals the handler is currently subscribed to. Updated
    dynamically when the enabled / disabled state or the primary clock changes.
    """

    _primary_seconds_for_zero_secondary_seconds: float = 0
    """Number of seconds on the primary clock that should correspond to zero
    seconds on the secondary clock.
    """

    end_of_show_seconds: float | None = None
    """The number of seconds on the secondary clock when the show ends and the
    secondary clock stops freewheeling; ``None`` if unknown, in which case it
    freewheels until it is synchronized to a different start.
    """

    log: Logger | None = None
    """Logger to use for logging messages. If ``None``, no logging will be done
    by the handler.
    """

    notify: Callable[[str], None] | None = None
    """Function that forwards a warning to the operator when a change of the
    primary clock is ignored because the secondary clock is freewheeling.
    """

    notify_late_start: Callable[[str], None] | None = None
    """Function that forwards a warning to the operator when the start becomes
    known through the primary clock less than `MIN_START_LEAD` seconds before
    it; ``None`` if no one needs the start that early.
    """

    point_of_no_return_seconds: float | None = None
    """The number of seconds on the secondary clock that correspond to the
    so-called "point of no return".
    """

    def __init__(
        self,
        *,
        point_of_no_return_seconds: float | None = None,
    ):
        """Constructor.

        Parameters:
            point_of_no_return_seconds: the number of seconds on the secondary
                clock that correspond to the so-called "point of no return"
        """
        self.point_of_no_return_seconds = point_of_no_return_seconds

    @property
    def enabled(self) -> bool:
        """Whether the synchronization mechanism is enabled. It is guaranteed
        that the secondary clock will not be adjusted by the synchronization
        mechanism when it is disabled.
        """
        return self._enabled

    @property
    def freewheeling(self) -> bool:
        """Whether the running secondary clock is between the point of no
        return and the end of the show, where it ignores the primary clock.
        """
        clock = self._secondary_clock
        if (
            not self._enabled
            or self.point_of_no_return_seconds is None
            or clock is None
            or not clock.running
        ):
            return False
        seconds = clock.seconds
        return seconds >= self.point_of_no_return_seconds and (
            self.end_of_show_seconds is None or seconds < self.end_of_show_seconds
        )

    def is_freewheeling_on(self, clock_id: str, seconds: float) -> bool:
        """Whether the secondary clock is freewheeling on a synchronization to
        the clock with the given ID that puts zero seconds on the secondary
        clock at the given number of seconds on that clock.
        """
        return (
            self.freewheeling
            and self._primary_clock is not None
            and self._primary_clock.id == clock_id
            and self._primary_seconds_for_zero_secondary_seconds == seconds
        )

    @property
    def primary_clock(self) -> Clock | None:
        """The primary clock; ``None`` means that it has not been assigned yet and
        the secondary clock should be stopped.
        """
        return self._primary_clock

    @property
    def secondary_clock(self) -> TimeElapsedSinceReferenceClock | None:
        """The secondary clock; ``None`` means that it has not been assigned
        yet.

        Changing the clock while the synchronization is active will _not_ reset
        the state of the old clock as it is deassigned, but it _will_ update the
        state of the new clock immediately.
        """
        return self._secondary_clock

    @secondary_clock.setter
    def secondary_clock(self, value: TimeElapsedSinceReferenceClock | None) -> None:
        if self._secondary_clock == value:
            return

        self._secondary_clock = value
        self._update_secondary_clock()

    def disable(self) -> None:
        """Disables the synchronization mechanism, but does not make any changes
        to the configuration of the secondary clock.
        """
        self._primary_clock = None
        self._primary_seconds_for_zero_secondary_seconds = 0.0
        self._subscribe_to_or_unsubscribe_from_primary_clock()
        self._enabled = False

    def disable_and_stop(self) -> None:
        """Disables the synchronization mechanism and stops the secondary
        clock.
        """
        self.disable()
        if self._secondary_clock:
            self._secondary_clock.reference_time = None

    def synchronize_to(self, clock: Clock, seconds: float) -> None:
        """Enables the synchronization mechanism and attaches it to the given
        primary clock.

        Parameters:
            clock: the primary clock to synchronize to
            seconds: the number of seconds on the primary clock that should
                belong to zero seconds in the secondary clock
        """
        if self.is_freewheeling_on(clock.id, seconds):
            self._primary_clock = clock
            self._subscribe_to_or_unsubscribe_from_primary_clock()
            return

        self._enabled = True
        self._primary_clock = clock
        self._primary_seconds_for_zero_secondary_seconds = seconds
        self._subscribe_to_or_unsubscribe_from_primary_clock()
        self._update_secondary_clock()

    @contextmanager
    def use_secondary_clock(
        self, clock: TimeElapsedSinceReferenceClock
    ) -> Iterator[None]:
        """Context manager that assigns the given clock as a secondary clock
        to the synchronization object when entering the context and that
        detaches the clock when exiting the context.
        """
        if self.secondary_clock is not None:
            raise RuntimeError(
                "no secondary clock must be attached to the synchronization handler yet"
            )

        self.secondary_clock = clock
        try:
            yield
        finally:
            self.secondary_clock = None

    def _calculate_desired_state_of_secondary_clock(
        self, now: float
    ) -> tuple[bool, float | None]:
        """Calculates the desired state of the secondary clock, given the
        primary clock. Assumes that the synchronization mechanism is enabled.

        Parameters:
            now: the current timestamp

        Returns:
            a tuple containing whether the secondary clock should be running
            and the number of _seconds_ that the secondary clock should display
        """
        if not self._primary_clock:
            return (False, 0)

        ticks_on_primary = self._primary_clock.ticks_given_time(now)
        time_diff_sec = (
            ticks_on_primary / self._primary_clock.ticks_per_second
            - self._primary_seconds_for_zero_secondary_seconds
        )
        return (self._primary_clock.running, time_diff_sec)

    def _on_primary_clock_changed(self, sender: Any = None, **kwds):
        """Event handler that is called when the primary clock has been started,
        stopped, adjusted or reassigned.

        Updates the secondary clock to reflect the current state of the primary
        clock, except while the secondary clock is freewheeling.
        """
        if self.freewheeling:
            self._report_ignored_change(kwds.get("delta"))
        else:
            self._update_secondary_clock()

    def _report_late_start(self, lead: float) -> None:
        primary, secondary = self._primary_clock, self._secondary_clock
        if (
            self.notify_late_start is None
            or primary is None
            or secondary is None
            or lead >= MIN_START_LEAD
        ):
            return

        when = f"{lead:.1f} s before" if lead >= 0 else f"{-lead:.1f} s after"
        message = (
            f"Clock {primary.id!r} gave the start of clock {secondary.id!r} only "
            f"{when} it; drones need it at least {MIN_START_LEAD:.0f} s ahead"
        )
        if self.log:
            self.log.warning(message)
        self.notify_late_start(message)

    def _report_ignored_change(self, delta: float | None) -> None:
        primary, secondary = self._primary_clock, self._secondary_clock
        if primary is None or secondary is None:
            return

        if not primary.running:
            change = "stopped"
        elif delta is not None and (
            abs(delta) / primary.ticks_per_second >= REPORTED_ADJUSTMENT
        ):
            change = f"jumped by {delta / primary.ticks_per_second:+.2f} s"
        else:
            if self.log:
                self.log.info(
                    f"Clock {secondary.id!r} is past the point of no return, "
                    f"ignoring a change of clock {primary.id!r}"
                )
            return

        message = (
            f"Clock {primary.id!r} {change} after the point of no return; "
            f"clock {secondary.id!r} keeps running on the server clock"
        )
        if self.log:
            self.log.warning(message)
        if self.notify:
            self.notify(message)

    def _subscribe_to_or_unsubscribe_from_primary_clock(self):
        target_clock = self._primary_clock if self._enabled else None
        if target_clock == self._subscribed_clock:
            return

        if self._subscribed_clock:
            self._subscribed_clock.started.disconnect(
                self._on_primary_clock_changed,
                sender=self._subscribed_clock,
            )
            self._subscribed_clock.stopped.disconnect(
                self._on_primary_clock_changed,
                sender=self._subscribed_clock,
            )
            self._subscribed_clock.changed.disconnect(
                self._on_primary_clock_changed,
                sender=self._subscribed_clock,
            )

        self._subscribed_clock = target_clock

        if self._subscribed_clock:
            self._subscribed_clock.started.connect(
                self._on_primary_clock_changed,
                sender=self._subscribed_clock,
            )
            self._subscribed_clock.stopped.connect(
                self._on_primary_clock_changed,
                sender=self._subscribed_clock,
            )
            self._subscribed_clock.changed.connect(
                self._on_primary_clock_changed,
                sender=self._subscribed_clock,
            )

    def _update_secondary_clock(self) -> None:
        """Updates the start time (epoch) of the secondary clock, i.e. the
        time instant when the clock is supposed to show zero ticks. Does nothing
        if there is no secondary clock or if the synchronization handler is
        disabled.
        """
        if not self._enabled or self._secondary_clock is None:
            return

        now = time()
        (
            should_run,
            time_on_secondary_clock,
        ) = self._calculate_desired_state_of_secondary_clock(now)

        if time_on_secondary_clock is None:
            reference_time = None
        else:
            reference_time = now - time_on_secondary_clock

        start_was_known = self._secondary_clock.reference_time is not None
        self._secondary_clock.reference_time = reference_time if should_run else None
        if should_run and reference_time is not None and not start_was_known:
            self._report_late_start(reference_time - now)
