"""A tag's anchor table as its telemetry gives it.

A tag keeps no anchor list: it builds its table from the positions the
anchors advertise (rtls-link-zephyr ``docs/reference/anchor-geometry.md``,
"The tag's table" and "Tag telemetry"). Its 2 Hz health report says how many
rows it uses (``anuse``), how many anchors it hears and leaves out
(``anexc``), the drift of the live ranges from the advertised positions
(``sdrift``) and, one anchor per report in rotation, who holds which row
(``antbl``). The server assembles the table over the rotation.

The packing of ``antbl`` and its states are the SDK's
(``rtlslink.protocol``).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from rtlslink.protocol import ANCHOR_TABLE_STATES, decode_anchor_table_entry

__all__ = ("IN_USE", "TABLE_STAT", "TAG_SURVEY_STATS", "AnchorTable", "TableEntry")

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
