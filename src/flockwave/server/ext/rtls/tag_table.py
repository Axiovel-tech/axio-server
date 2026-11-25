"""A tag's anchor table as its telemetry gives it.

A tag keeps no anchor list: it builds its table from the positions the
anchors advertise (rtls-link-zephyr ``docs/reference/anchor-geometry.md``,
"The tag's table" and "Tag telemetry"). Its 2 Hz health report says how many
rows it uses (``anuse``), how many anchors it hears and leaves out
(``anexc``), the drift of the live ranges from the advertised positions
(``sdrift``) and, one anchor per report in rotation, who holds which row
(``antbl``). The server assembles the table over the rotation.

This module is also the one place that names what the SDK gains with the
tag's switch (the four stats and the ``antbl`` packing), so that a pin
without them still imports.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Optional

from rtlslink import protocol as _sdk

_STATES = {
    1: "in-use",
    2: "other-survey",
    3: "no-position",
    4: "unknown-layout",
    5: "table-full",
}

_NO_ROW = 15


def _decode(value: float) -> dict[str, int | None] | None:
    """One ``antbl`` stat (``row * 2**20 + state * 2**16 + mac``) as ``mac``,
    ``state`` and ``row``, which is None for an anchor the tag leaves out;
    None for 0 and for a value that is no entry."""
    entry = int(value)
    if entry != value or not 0 < entry < 1 << 24:
        return None
    row, state, mac = entry >> 20, (entry >> 16) & 0xF, entry & 0xFFFF
    if state not in _STATES or (state == 1) != (row != _NO_ROW):
        return None
    return {"mac": mac, "state": state, "row": row if state == 1 else None}


#: the SDK's own once the pin has them
ANCHOR_TABLE_STATES: dict[int, str] = getattr(_sdk, "ANCHOR_TABLE_STATES", _STATES)
decode_anchor_table_entry: Callable[[float], Optional[dict[str, Optional[int]]]] = (
    getattr(_sdk, "decode_anchor_table_entry", _decode)
)

__all__ = (
    "ANCHOR_TABLE_STATES",
    "IN_USE",
    "TABLE_STAT",
    "TAG_SURVEY_STATS",
    "AnchorTable",
    "TableEntry",
    "decode_anchor_table_entry",
)

#: wire name, X-RTLS-STATS key and decimals (None: an integer) of the stats
#: a tag reports of its table besides ``sid``
TAG_SURVEY_STATS: tuple[tuple[str, str, Optional[int]], ...] = (
    ("anuse", "anchorsUsed", None),
    ("anexc", "anchorsExcluded", None),
    ("sdrift", "surveyDriftM", 4),
)

TABLE_STAT = "antbl"

IN_USE = ANCHOR_TABLE_STATES[1]

#: an entry leaves the table when this many rotations passed without it
ROTATIONS_KEPT = 2


@dataclass(frozen=True)
class TableEntry:
    """An anchor a tag tracks."""

    mac: int
    #: an ``ANCHOR_TABLE_STATES`` name
    state: str
    #: the tag's row of an anchor in use, None for one it leaves out
    row: Optional[int]


class AnchorTable:
    """One tag's table, assembled from the one entry each report names."""

    def __init__(self) -> None:
        self._entries: dict[int, TableEntry] = {}
        #: the report that last named each anchor
        self._named_at: dict[int, int] = {}
        self._reports = 0

    @property
    def entries(self) -> list[TableEntry]:
        """The rows in use by row, then the anchors left out by MAC."""
        return sorted(
            self._entries.values(),
            key=lambda entry: (entry.row is None, entry.row or 0, entry.mac),
        )

    def report(self, value: float) -> None:
        """Takes the ``antbl`` of one report: 0 empties the table, an entry
        replaces what its anchor and its row held, and an anchor the
        rotation stopped naming leaves."""
        self._reports += 1
        if value == 0:
            self._entries.clear()
            self._named_at.clear()
            return
        decoded = decode_anchor_table_entry(value)
        if decoded is not None:
            self._name(decoded["mac"] or 0, decoded["state"] or 0, decoded["row"])
        kept = ROTATIONS_KEPT * len(self._entries)
        for mac, named_at in list(self._named_at.items()):
            if self._reports - named_at > kept:
                self._forget(mac)

    def _name(self, mac: int, state: int, row: Optional[int]) -> None:
        if row is not None:
            for other in [e.mac for e in self._entries.values() if e.row == row]:
                self._forget(other)
        self._entries[mac] = TableEntry(mac, ANCHOR_TABLE_STATES[state], row)
        self._named_at[mac] = self._reports

    def _forget(self, mac: int) -> None:
        self._entries.pop(mac, None)
        self._named_at.pop(mac, None)
