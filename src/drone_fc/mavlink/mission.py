"""航线点校验与 degE7 编码，不依赖 pymavlink 或真机。"""

from __future__ import annotations

from collections.abc import Sequence

from drone_fc.exceptions import InvalidArgumentError
from drone_fc.types import MissionItem

MAX_MISSION_ITEMS = 128
_ALT_MAX_M = 500.0


def deg_to_degE7(degrees: float) -> int:
    """经纬度（度）→ MISSION_ITEM_INT / GLOBAL 用的 degE7。

    输入:
        degrees: 有限浮点，纬度约 [-90, 90]，经度约 [-180, 180]。
    输出:
        四舍五入后的整数，等于 degrees × 1e7。
    """

    return int(round(float(degrees) * 1e7))


def validate_mission_items(items: Sequence[MissionItem]) -> tuple[MissionItem, ...]:
    """检查航线列表长度与每个点的范围。

    输入:
        items: 非空序列，长度 [1, 128]。
    输出:
        规范化后的 tuple（每个字段为有限 float / int）。
    失败:
        InvalidArgumentError: 空、过长、坐标或高度非法。
    """

    if not isinstance(items, Sequence) or isinstance(items, (str, bytes)):
        raise InvalidArgumentError("mission items must be a sequence of MissionItem")
    if len(items) < 1 or len(items) > MAX_MISSION_ITEMS:
        raise InvalidArgumentError(
            f"mission length must be in [1, {MAX_MISSION_ITEMS}], got {len(items)}"
        )
    return tuple(_validate_one(item, index) for index, item in enumerate(items))


def _validate_one(item: MissionItem, index: int) -> MissionItem:
    """校验单点。index 仅用于错误信息。"""

    if not isinstance(item, MissionItem):
        raise InvalidArgumentError(f"mission item {index} is not a MissionItem")
    lat = _finite(f"item[{index}].latitude_deg", item.latitude_deg)
    lon = _finite(f"item[{index}].longitude_deg", item.longitude_deg)
    alt = _finite(f"item[{index}].altitude_m", item.altitude_m)
    hold = _finite(f"item[{index}].hold_s", item.hold_s)
    radius = _finite(f"item[{index}].accept_radius_m", item.accept_radius_m)
    if lat < -90.0 or lat > 90.0:
        raise InvalidArgumentError(f"item[{index}].latitude_deg must be in [-90, 90], got {lat}")
    if lon < -180.0 or lon > 180.0:
        raise InvalidArgumentError(f"item[{index}].longitude_deg must be in [-180, 180], got {lon}")
    if alt <= 0.0 or alt > _ALT_MAX_M:
        raise InvalidArgumentError(
            f"item[{index}].altitude_m must be in (0, {_ALT_MAX_M}], got {alt}"
        )
    if hold < 0.0:
        raise InvalidArgumentError(f"item[{index}].hold_s must be >= 0, got {hold}")
    if radius < 0.0:
        raise InvalidArgumentError(f"item[{index}].accept_radius_m must be >= 0, got {radius}")
    if not isinstance(item.command, int) or item.command < 0:
        raise InvalidArgumentError(f"item[{index}].command must be a MAV_CMD integer >= 0")
    return MissionItem(
        latitude_deg=lat,
        longitude_deg=lon,
        altitude_m=alt,
        command=int(item.command),
        hold_s=hold,
        accept_radius_m=radius,
    )


def _finite(name: str, value: float) -> float:
    if not isinstance(value, (int, float)) or value != value or abs(value) == float("inf"):
        raise InvalidArgumentError(f"{name} must be a finite number")
    return float(value)
