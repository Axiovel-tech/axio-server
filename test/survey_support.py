"""A surveyable cell for the survey tests: anchor_lock_support's cell with
anchors that behave like the firmware towards a survey. An anchor acks a
step command and answers with the report of that step, its ranges taken
from true positions; it stores ``UWB_LOC`` whole; and it reports its health,
the survey ID it runs included, when the test lets it."""

import math
import time

import trio
from anchor_lock_support import HEALTHY_A0, HEALTHY_A1, _cell
from rtls_support import make_message, set_fake_param
from rtlslink.anchor_location import decode_param
from rtlslink.protocol import PARAM_ACK_FAILED, PARAM_TYPE_CUSTOM, raw_field_bytes

#: four anchors at one height, system ID -> north, east, down in metres;
#: their MACs are 1..4 in this order and 201 is the stored initiator
FLAT = {
    201: (0.0, 0.0, 0.0),
    202: (4.0, 0.0, 0.0),
    203: (4.2, 3.1, 0.0),
    204: (0.3, 2.8, 0.0),
}

#: the standard cell: FLAT with four more anchors above it
TIERS = FLAT | {
    205: (0.1, 0.2, -2.4),
    206: (3.9, 0.1, -2.4),
    207: (4.1, 3.0, -2.4),
    208: (0.2, 2.9, -2.4),
}

STEP_PARAM = "UWB_SURVEY"
LOC_PARAM = "UWB_LOC"


def _text(message):
    value = raw_field_bytes(message, "param_value") or b""
    return value.split(b"\x00")[0].decode()


class SurveyCell:
    """The cell and the knobs of its anchors' behaviour."""

    def __init__(self, ext, devices, dialect, positions, nursery):
        self.ext = ext
        self.nursery = nursery
        self.devices = devices
        self.dialect = dialect
        self.positions = positions
        self.mac = {system_id: index + 1 for index, system_id in enumerate(positions)}
        #: every step command an anchor received: system ID, value, when
        self.commands = []
        #: every ``UWB_LOC`` value an anchor received
        self.locations = []
        #: anchors that refuse, or do not answer, a step command
        self.refusing = set()
        self.silent = set()
        #: which of the steps an anchor is commanded it refuses, from 1
        self.refused_steps = {}
        #: anchors whose radio does not start a step
        self.failing = set()
        #: anchors that report a range to another peer than the initiator
        self.misreporting = set()
        #: how many more step commands an anchor does not hear
        self.unheard = {}
        #: which of the steps an anchor is commanded it completes no report
        #: of, counted from 1
        self.lost_reports = {}
        self._steps_seen = {}
        self._reports_sent = 0
        #: how long an anchor's answer to a step command takes, in seconds
        self.late = {}
        #: how long an anchor takes to collect a step, in seconds
        self.collect_s = 0.0
        #: ``(initiator, responder)`` -> valid share, range error in metres
        self.share = {}
        self.error = {}
        #: anchors that refuse, or do not answer, a ``UWB_LOC`` write
        self.location_refusing = set()
        self.location_silent = set()
        #: stats an anchor reports besides the healthy ones
        self.stats = {}
        for device in devices.values():
            if device.system_id in positions:
                self._behave(device)

    @property
    def broadcasts(self):
        """The survey states the extension pushed."""
        sent = self.ext.app.message_hub.broadcasts
        return [body["survey"] for body in sent if body["type"] == "X-RTLS-SURVEY"]

    def steps(self):
        """The distinct step commands in the order they were first sent."""
        return list(dict.fromkeys(value for _, value, _ in self.commands))

    def sent(self, system_id, value):
        """When ``system_id`` received the step command ``value``."""
        return [
            at for to, sent, at in self.commands if (to, sent) == (system_id, value)
        ]

    def stored(self, system_id):
        """What an anchor stores in ``UWB_LOC``, decoded; None for a value
        that is no location."""
        try:
            return decode_param(self.devices[system_id].params[LOC_PARAM][0].decode())
        except ValueError:
            return None

    async def report(self, ids=None, **stats):
        """Each anchor of ``ids`` (default: all) sends its health report."""
        first = next(iter(self.positions))
        for system_id in self.positions if ids is None else ids:
            stored = self.stored(system_id)
            body = {
                **(HEALTHY_A0 if system_id == first else HEALTHY_A1),
                "sid": float(stored.location.survey_id if stored else 0),
                **self.stats.get(system_id, {}),
                **stats,
            }
            await self.ext._on_stats(system_id, body, time.monotonic())

    def lose(self, system_id):
        """The anchor's heartbeats stopped long ago: the extension loses it."""
        self.ext._protocol.devices[system_id].last_seen = -1e9
        self.ext._expire_devices(time.monotonic())

    def _behave(self, device):
        handle, parser = device.handle, self.dialect.MAVLink(None)

        def handled(data):
            message = (parser.parse_buffer(data) or [None])[0]
            if message is not None and message.get_type() == "PARAM_EXT_SET":
                if message.param_id == STEP_PARAM:
                    return self._step_command(device, _text(message))
                if message.param_id == LOC_PARAM:
                    return self._location_write(device, _text(message))
            return handle(data)

        device.handle = handled

    def _step_command(self, device, value):
        system_id = device.system_id
        self.commands.append((system_id, value, trio.current_time()))
        if system_id in self.silent or self.unheard.get(system_id, 0) > 0:
            self.unheard[system_id] = self.unheard.get(system_id, 0) - 1
            return []
        seen = self._steps_seen.setdefault(system_id, [])
        if value != "0000" and value[:4] not in seen:
            seen.append(value[:4])
        step = seen.index(value[:4]) + 1 if value != "0000" else 0
        if system_id in self.refusing or step in self.refused_steps.get(system_id, ()):
            return [
                device._param_ack(STEP_PARAM, b"", PARAM_TYPE_CUSTOM, PARAM_ACK_FAILED)
            ]
        replies = [device._param_ack(STEP_PARAM, value.encode(), PARAM_TYPE_CUSTOM, 0)]
        if value != "0000":
            if step in self.lost_reports.get(system_id, ()):
                pass
            elif self.collect_s:
                self._later(
                    device, self.collect_s, lambda: [self._report(device, value)]
                )
            else:
                replies.append(self._report(device, value))
        if system_id in self.late:
            self._later(device, self.late[system_id], lambda: replies)
            return []
        return replies

    def _later(self, device, delay, datagrams):
        async def send():
            await trio.sleep(delay)
            for datagram in datagrams():
                await self.ext._process_datagram(
                    datagram, device.address, time.monotonic()
                )

        self.nursery.start_soon(send)

    def _report(self, device, value):
        """The complete report of the step ``value`` commands, all frames in
        one datagram."""
        system_id = device.system_id
        step_id, rounds = int(value[:4], 16), int(value[4:8], 16)
        initiator_mac = int(value[10:14], 16)
        initiator = next(s for s, mac in self.mac.items() if mac == initiator_mac)
        frames: dict[str, float] = {
            "svid": step_id,
            "svrole": 3,
            "svst": 2,
            "svrnd": rounds,
        }
        if system_id in self.failing:
            frames["svst"] = 3
        elif system_id == initiator:
            frames |= {"svrole": 2, "svn": rounds}
        else:
            pair = (initiator, system_id)
            heard = initiator not in self.failing
            valid = round(self.share.get(pair, 0.98) * rounds) if heard else 0
            distance = math.dist(self.positions[initiator], self.positions[system_id])
            frames |= {
                "svpeer": 0x7777 if system_id in self.misreporting else initiator_mac,
                "svrng": distance + self.error.get(pair, 0.0) if valid else 0.0,
                "svmad": 0.004,
                "svn": valid,
                "svok": valid,
            }
        # the SDK takes the frames of one time_boot_ms for one report, once
        self._reports_sent += 1
        return b"".join(
            device.named_value_float(name, float(value), self._reports_sent)
            for name, value in frames.items()
        )

    def _location_write(self, device, value):
        system_id = device.system_id
        self.locations.append((system_id, value))
        if system_id in self.location_silent:
            return []
        if system_id in self.location_refusing:
            current = device.params[LOC_PARAM][0]
            return [
                device._param_ack(
                    LOC_PARAM, current, PARAM_TYPE_CUSTOM, PARAM_ACK_FAILED
                )
            ]
        device.params[LOC_PARAM] = (value.encode(), PARAM_TYPE_CUSTOM)
        return [device._param_ack(LOC_PARAM, value.encode(), PARAM_TYPE_CUSTOM, 0)]


async def survey_cell(dialect, nursery, positions=FLAT):
    """An extension with the anchors of ``positions`` discovered, healthy and
    storing one cell in this order, its survey runs started in ``nursery``."""
    anchors = tuple(
        (system_id, 2 if index == 0 else 3, index + 1)
        for index, system_id in enumerate(positions)
    )

    def prepare(devices):
        for system_id in positions:
            device = devices[system_id]
            set_fake_param(device, "UWB_AN_COUNT", len(positions), "uint8")
            for index in range(len(positions)):
                set_fake_param(device, f"UWB_AN{index}_MAC", index + 1, "uint16")
            set_fake_param(device, LOC_PARAM, "", "custom")
            set_fake_param(device, STEP_PARAM, "", "custom")

    ext, devices = await _cell(dialect, anchors=anchors, prepare=prepare)
    ext._nursery = nursery
    cell = SurveyCell(ext, devices, dialect, positions, nursery)
    await cell.report()
    return cell


async def request(cell, builder, hub, action=None, **body):
    """The body of the answer to an X-RTLS-SURVEY request."""
    if action is not None:
        body["action"] = action
    message = make_message(builder, {"type": "X-RTLS-SURVEY"} | body)
    response = await cell.ext._handle_RTLS_SURVEY(message, None, hub)
    return response.body


TWO_TIERS = {"preset": "two-tiers"}


def authorize_show(ext):
    """The show extension reports a flight authorized."""
    from flockwave.server.ext.show.config import AuthorizationScope

    class Show:
        authorized_to_start = True
        authorization_scope = AuthorizationScope.REHEARSAL

    class Api:
        def get_configuration(self):
            return Show()

    ext.app.import_api = lambda name: Api()


async def until(cell, *states):
    """Waits until the survey is in one of ``states``."""
    with trio.fail_after(300):
        while cell.ext._survey.state not in states:
            await trio.sleep(0.01)


async def measure(cell, builder, hub, **body):
    """A whole run of the cell's anchors on the standard frame: started, its
    anchors reporting ranging once it returns them; the state it ends in."""
    body = {"anchors": list(cell.positions), "frame": TWO_TIERS} | body
    answer = await request(cell, builder, hub, "start", **body)
    assert answer["result"] == "started", answer["reason"]
    await until(cell, "returning")
    await cell.report()
    await until(cell, "ready", "failed", "idle")
    return cell.ext._survey.json


async def commit(cell, builder, hub, reporting=None, after=0.1, **body):
    """The answer to a commit, during which the anchors of ``reporting``
    (default: all) report their health once the writes are out, ``after``
    seconds into it."""
    answer = {}

    async def ask():
        answer.update(await request(cell, builder, hub, "commit", **body))

    async with trio.open_nursery() as nursery:
        nursery.start_soon(ask)
        await trio.sleep(after)
        await cell.report(reporting)
    return answer
