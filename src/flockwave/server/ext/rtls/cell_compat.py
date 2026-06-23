"""Compatibility wrapper for RTLS cell helpers.

The preferred implementation lives in the ``rtlslink`` SDK. The fallback keeps
this server branch testable until its SDK dependency is advanced to a version
that contains those helpers.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping

try:
    from rtlslink import cell_from_params, ned_to_global_e7, role_from_params
except ImportError:
    MAX_ANCHORS = 8
    METERS_PER_LAT_E7 = 0.0111319490
    ROLE_NAMES = {
        0: "disabled",
        1: "tag",
        2: "anchor-initiator",
        3: "anchor-responder",
    }

    @dataclass(frozen=True)
    class Origin:
        lat_e7: int
        lon_e7: int
        alt_mm: int

        @property
        def lat_deg(self) -> float:
            return self.lat_e7 * 1e-7

    @dataclass(frozen=True)
    class Anchor:
        index: int
        north_m: float
        east_m: float
        down_m: float
        mac: int | None = None
        bias_m: float = 0.0

    @dataclass(frozen=True)
    class RtlsCell:
        origin: Origin
        anchors: tuple[Anchor, ...] = field(default_factory=tuple)
        cell_id: str = "default"

        def __init__(
            self,
            origin: Origin,
            anchors: Iterable[Anchor] = (),
            *,
            cell_id: str = "default",
        ):
            object.__setattr__(self, "origin", origin)
            object.__setattr__(self, "anchors", tuple(anchors))
            object.__setattr__(self, "cell_id", cell_id)

    def _anchor_param_name(index: int, suffix: str) -> str:
        return f"UWB_AN{index}_{suffix}"

    def _param_value(
        params: Mapping[str, Any], name: str, *, default: Any = ...
    ) -> Any:
        if name not in params:
            if default is ...:
                raise KeyError(name)
            return default
        value = params[name]
        if isinstance(value, Mapping) and "value" in value:
            return value["value"]
        return value

    def role_from_params(params: Mapping[str, Any]) -> str | None:
        value = _param_value(params, "UWB_ROLE", default=None)
        if value is None:
            return None
        try:
            return ROLE_NAMES.get(int(value))
        except (TypeError, ValueError):
            return None

    def cell_from_params(
        params: Mapping[str, Any], *, cell_id: str = "default"
    ) -> RtlsCell:
        origin = Origin(
            lat_e7=int(_param_value(params, "ORIGIN_LAT_E7")),
            lon_e7=int(_param_value(params, "ORIGIN_LON_E7")),
            alt_mm=int(_param_value(params, "ORIGIN_ALT_MM")),
        )
        count = int(_param_value(params, "UWB_AN_COUNT"))
        if count < 0 or count > MAX_ANCHORS:
            raise ValueError(f"anchor count must be between 0 and {MAX_ANCHORS}")

        anchors = []
        for index in range(count):
            mac = _param_value(params, _anchor_param_name(index, "MAC"), default=None)
            bias = _param_value(
                params, _anchor_param_name(index, "BIAS_M"), default=0.0
            )
            anchors.append(
                Anchor(
                    index=index,
                    north_m=float(_param_value(params, _anchor_param_name(index, "X"))),
                    east_m=float(_param_value(params, _anchor_param_name(index, "Y"))),
                    down_m=float(_param_value(params, _anchor_param_name(index, "Z"))),
                    mac=None if mac is None else int(mac),
                    bias_m=float(bias),
                )
            )
        return RtlsCell(origin, anchors, cell_id=cell_id)

    def ned_to_global_e7(
        origin: Origin, north_m: float, east_m: float, down_m: float
    ) -> tuple[int, int, int]:
        lon_scale = METERS_PER_LAT_E7 * math.cos(math.radians(origin.lat_deg))
        if abs(lon_scale) < 1e-9:
            raise ValueError("origin latitude is too close to a pole")
        return (
            origin.lat_e7 + round(float(north_m) / METERS_PER_LAT_E7),
            origin.lon_e7 + round(float(east_m) / lon_scale),
            origin.alt_mm - round(float(down_m) * 1000.0),
        )


__all__ = ("cell_from_params", "ned_to_global_e7", "role_from_params")
