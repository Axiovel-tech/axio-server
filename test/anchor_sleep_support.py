"""A scripted anchor for the X-RTLS-SLEEP tests: it reboots, falls asleep
in place or refuses like the firmware's power path."""

import time

import trio
import trio.testing
from rtls_support import (
    DEVICE_SYSID,
    FakeDevice,
    StubApp,
    discover,
    make_message,
    set_fake_param,
)
from rtlslink.protocol import ANCHOR_POWER_STATES, PARAM_TYPE_UINT8, RtlsProtocol

from flockwave.server.ext.rtls.extension import DEFAULT_DEVICE_TIMEOUT, RtlsExtension

#: pwrst code per power state name
POWER_CODES = {name: code for code, name in ANCHOR_POWER_STATES.items()}


class FakeAnchor(FakeDevice):
    """Scripted anchor power path: a SLEEP change is acked, then the anchor
    reboots into the requested state, off the network for ``reboot_s``
    seconds, like rtls-link-zephyr's anchor (docs/reference/
    anchor-power-path.md). ``refuse_with`` makes it refuse instead,
    reading back as that SLEEP value; ``hang`` accepts and never reboots;
    ``boot_s`` keeps it ``booting`` (SLEEP 0) that long after the reboot.
    With ``in_place_sleep`` a sleep request in plain standby moves it to
    asleep without a reboot; it streams ``pwrst`` unless
    ``reports_power_state`` is off."""

    def __init__(
        self,
        dialect,
        *,
        standby=False,
        power=None,
        reboot_s=9.0,
        in_place_sleep=False,
        reports_power_state=True,
        sleep_value=None,
        boot_s=0.0,
    ):
        super().__init__(dialect)
        set_fake_param(self, "UWB_ROLE", 3, "uint8")
        if sleep_value is None:
            sleep_value = 1 if standby else 0
        self.params["SLEEP"] = (bytes([sleep_value]), PARAM_TYPE_UINT8)
        self.power = power or ("standby" if standby else "operating")
        self.in_place_sleep = in_place_sleep
        self.reports_power_state = reports_power_state
        self.reboot_s = reboot_s
        self.boot_s = boot_s
        self.booted_at = None
        self.reboot_at = None
        self.target = None
        self.refuse_with = None
        self.hang = False
        self.expired = False

    @property
    def sleeping(self) -> bool:
        return self.power in ("standby", "asleep", "off")

    @property
    def down(self) -> bool:
        return self.reboot_at is not None and trio.current_time() >= self.reboot_at

    def handle(self, data: bytes) -> list[bytes]:
        return [] if self.down else super().handle(data)

    def after_sleep_write(self, previous: bytes, value: bytes) -> None:
        asleep = value == b"\x01"
        if self.refuse_with is not None:
            self.params["SLEEP"] = (bytes([self.refuse_with]), PARAM_TYPE_UINT8)
        elif asleep and self.power == "standby" and self.in_place_sleep:
            self.power = "asleep"
        elif value != previous and not self.hang:
            self.target = "asleep" if asleep else "operating"
            self.reboot_at = trio.current_time() + 0.5

    async def run(self, extension):
        """Heartbeats once a second while up; the server loop's expiry is
        emulated once the reboot outlasts the device timeout."""
        while True:
            now = trio.current_time()
            if self.down:
                if not self.expired and now - self.reboot_at > DEFAULT_DEVICE_TIMEOUT:
                    self.expired = True
                    extension._expire_devices(time.monotonic() + 1000.0)
                if now >= self.reboot_at + self.reboot_s:
                    self.power = "booting" if self.boot_s else self.target
                    self.booted_at = now
                    self.reboot_at = None
                    if self.power == "booting":
                        # until the retained intent applies (firmware sleep_value)
                        self.params["SLEEP"] = (bytes([0]), PARAM_TYPE_UINT8)
                    elif self.power == "off":
                        self.params["SLEEP"] = (bytes([2]), PARAM_TYPE_UINT8)
            elif self.power == "booting" and now >= self.booted_at + self.boot_s:
                self.power = self.target
            if not self.down:
                await extension._process_datagram(
                    self.heartbeat(), self.address, time.monotonic()
                )
                if self.reports_power_state:
                    code = POWER_CODES[self.power]
                    await extension._process_datagram(
                        self.named_value_float("pwrst", float(code)),
                        self.address,
                        time.monotonic(),
                    )
            await trio.sleep(1.0)


async def _anchor_extension(dialect, anchor):
    """An extension wired to ``anchor`` that has discovered it."""
    ext = RtlsExtension()
    ext._protocol = RtlsProtocol(dialect, targets=[anchor.address])
    ext.app = StubApp()

    async def fake_send(payload, address):
        for reply in anchor.handle(payload):
            await ext._process_datagram(reply, anchor.address, time.monotonic())

    ext._send = fake_send
    await discover(ext, anchor)
    return ext


async def _send_sleep(ext, builder, hub, sleeping, **body):
    response = await ext._handle_RTLS_SLEEP(
        make_message(
            builder,
            {"type": "X-RTLS-SLEEP", "ids": [DEVICE_SYSID], "sleeping": sleeping}
            | body,
        ),
        None,
        hub,
    )
    return response.body["result"][str(DEVICE_SYSID)]


async def _anchor_sleep(dialect, builder, hub, anchor, sleeping, **body):
    """Discover the anchor, run it until it has reported its powerState, and
    send it one X-RTLS-SLEEP; returns the extension, the anchor's result
    entry and the seconds the request took."""
    ext = await _anchor_extension(dialect, anchor)
    async with trio.open_nursery() as nursery:
        nursery.start_soon(anchor.run, ext)
        await trio.sleep(1.5)
        if anchor.reports_power_state:
            assert ext._power_state(DEVICE_SYSID) == anchor.power
        started = trio.current_time()
        entry = await _send_sleep(ext, builder, hub, sleeping, **body)
        elapsed = trio.current_time() - started
        nursery.cancel_scope.cancel()
    return ext, entry, elapsed
