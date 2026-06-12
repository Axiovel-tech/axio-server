#!/usr/bin/env python3
"""Standalone verification of the sans-IO rtls protocol core against a
LIVE rtls-link firmware (native_sim or hardware on the same network).

Run as a plain file (no flockwave install needed; uses pymavlink):

    python3 selfcheck.py --host 127.0.0.1 --port 3333

Asserts: device discovery via heartbeat exchange, full PARAM_EXT list,
and a set/ack roundtrip. Exit 0 on PASS.
"""

import argparse
import importlib.util
import os
import socket
import sys
import time

os.environ.setdefault("MAVLINK20", "1")


def load_protocol_module():
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "protocol.py")
    spec = importlib.util.spec_from_file_location("rtls_protocol", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules["rtls_protocol"] = module  # dataclasses resolve via sys.modules
    spec.loader.exec_module(module)
    return module


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=3333)
    ap.add_argument("--timeout", type=float, default=20.0)
    args = ap.parse_args()

    from pymavlink.dialects.v20 import ardupilotmega as dialect

    rtls = load_protocol_module()
    protocol = rtls.RtlsProtocol(
        dialect, targets=[(args.host, args.port)], heartbeat_interval=1.0
    )

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)

    discovered = None
    params = {}
    expected = None
    ack = None
    list_requested = False
    set_sent = False

    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        now = time.monotonic()
        for payload, address in protocol.poll(now):
            sock.sendto(payload, address)
        try:
            data, address = sock.recvfrom(4096)
        except BlockingIOError:
            time.sleep(0.05)
            continue
        for event in protocol.feed(data, address, now):
            if event.kind == "discovered":
                discovered = event.system_id
                print(f"discovered sysid {event.system_id} at {event.data['address']}")
                request = protocol.request_param_list(event.system_id)
                sock.sendto(*request)
                list_requested = True
            elif event.kind == "param_value":
                params[event.data["name"]] = event.data["value"]
                expected = event.data["count"]
            elif event.kind == "param_ack":
                ack = event.data
                print(f"ack: {ack['name']} result={ack['result']}")

        if list_requested and not set_sent and expected and len(params) >= expected:
            print(f"param list complete: {len(params)} parameters")
            assert "MAV_SYS_ID" in params, sorted(params)
            current = params["MAV_SYS_ID"][0]
            request = protocol.set_param(
                discovered, "MAV_SYS_ID", bytes([current]), rtls.PARAM_TYPE_UINT8
            )
            sock.sendto(*request)
            set_sent = True

        if set_sent and ack is not None:
            assert ack["name"] == "MAV_SYS_ID" and ack["result"] == 0, ack
            print("PASS")
            return 0

    print(
        "FAIL: timed out",
        f"discovered={discovered}",
        f"params={len(params)}/{expected}",
        f"ack={ack}",
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
