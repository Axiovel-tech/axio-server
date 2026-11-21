"""The anchor cell lock's state on its own (rtls/anchor_lock.py) and the
lock warnings: the integrity watch's rules, persistence and the uptime
arithmetic."""

from flockwave.server.ext.rtls.anchor_lock import AnchorLock
from flockwave.server.ext.rtls.anchor_lock_gate import cell_warnings


def test_cell_warnings():
    def view(system_id, role, online=True, **stats):
        return {
            "systemId": system_id,
            "name": f"A{system_id}",
            "role": role,
            "online": online,
            "stats": stats,
        }

    warnings = cell_warnings(
        [
            view(1, "anchor-initiator", powerState="standby"),
            view(2, "anchor-responder", rangingRateHz=0.0, moved=True),
            view(3, "anchor-responder", uwbStatus=3, radioRestarts=2),
            view(4, "anchor-responder", online=False),
        ]
    )
    assert [(w["code"], w["device"]) for w in warnings] == [
        ("a0-not-operating", 1),
        ("a0-not-heard", 2),
        ("moved", 2),
        ("uwb-start-refused", 3),
        ("radio-restarts", 3),
        ("offline", 4),
    ]
    # an initiator that is offline, or missing, is a cell warning
    assert cell_warnings([view(2, "anchor-responder")])[0]["code"] == (
        "a0-not-operating"
    )


def test_the_uptime_wrap_is_no_reboot():
    lock = AnchorLock()
    lock.seal(
        {201: _sealed(uptimeMs=2**32 - 4_000)},
        [],
        1000.0,
        {201: 0.0},
    )
    # 10 s later the 32-bit uptime wrapped to 6 s
    assert lock.on_uptime(201, 6_000, 10.0, 1010.0) == []
    assert [b["code"] for b in lock.on_uptime(201, 1_000, 20.0, 1020.0)] == ["rebooted"]


def _sealed(**changes):
    entry = {
        "name": "RTLS anchor A0",
        "role": "anchor-initiator",
        "firmware": "1.2.3",
        "params": {"UWB_CHANNEL": 9},
        "bootCount": 3,
        "uptimeMs": None,
        "powerState": "operating",
        "moved": False,
        "online": True,
    }
    entry.update(changes)
    return entry


def test_lock_persists_across_a_restart(tmp_path):
    path = tmp_path / "anchor_lock.json"
    lock = AnchorLock(path)
    lock.seal(
        {
            201: {
                "name": "RTLS anchor A0",
                "role": "anchor-initiator",
                "firmware": "1.2.3",
                "params": {"UWB_CHANNEL": 9},
                "bootCount": 3,
                "uptimeMs": None,
                "powerState": "operating",
                "moved": False,
                "online": True,
            }
        },
        [],
        1000.0,
    )
    lock.on_stats(201, {"moved": True}, 1001.0)

    lock.on_param(201, "UWB_CHANNEL", 7, 1001.5)

    restored = AnchorLock(path)
    restored.load(6.0, 0.0)
    assert restored.json() == lock.json()
    assert restored.covers(201)
    # the last compared values survive: the change already recorded does
    # not break again, a change back while the server was down does
    restored.on_param(201, "UWB_CHANNEL", 7, 1002.0)
    restored.on_param(201, "UWB_CHANNEL", 9, 1002.0)
    restored.on_stats(201, {"bootCount": 4}, 1002.0)
    assert [b["code"] for b in reversed(restored.breaks)] == [
        "moved",
        "param-changed",
        "param-changed",
        "rebooted",
    ]


def test_a_late_advert_is_no_reboot():
    lock = AnchorLock()
    lock.seal({201: _sealed(uptimeMs=100_000)}, [], 1000.0, {201: 0.0})
    # one advert arrives 2 s late, the next one on time
    assert lock.on_uptime(201, 110_000, 12.0, 1012.0) == []
    assert lock.on_uptime(201, 120_000, 20.0, 1020.0) == []
    # an uptime that fell behind by more than the slack is a reboot
    assert [b["code"] for b in lock.on_uptime(201, 3_000, 40.0, 1040.0)] == ["rebooted"]
