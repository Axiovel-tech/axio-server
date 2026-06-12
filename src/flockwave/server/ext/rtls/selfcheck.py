#!/usr/bin/env python3
"""Deprecated shim; the live-firmware protocol selfcheck moved to the
standalone ``rtls-link`` SDK. Use the CLI instead:

    rtls-link selfcheck <host>[:<port>]

This file only keeps old invocations working by forwarding to
:func:`rtlslink.selfcheck.run`.
"""

import argparse
import sys


def main() -> int:
    print(
        "selfcheck.py is deprecated; use `rtls-link selfcheck <host>[:<port>]` "
        "from the rtls-link SDK instead",
        file=sys.stderr,
    )

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=3333)
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()

    from rtlslink.selfcheck import run

    return run(args.host, args.port, args.timeout)


if __name__ == "__main__":
    sys.exit(main())
