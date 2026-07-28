"""通信协议编解码（ADR-004 适配版）。

50 字节/消息，纯 ASCII 编码，每条目标 ~18 bytes：
  "T:lat,lon"  （T=类型, lat/lon=3位小数, 精度~110m）

最多 2 条目标/消息。与 baseline coop_distributed 格式兼容。
"""

from dataclasses import dataclass

_MAX_PAYLOAD_BYTES = 50

VALID_TYPES = frozenset("TDAJ")
VALID_STATUSES = frozenset(range(3))


@dataclass
class TargetInfo:
    """单条目标信息。"""

    type: str  # 'T'=真目标, 'D'=诱饵, 'A'=认领, 'J'=干扰
    target_id: int  # 0-255
    lat: float  # WGS84 纬度
    lon: float  # WGS84 经度
    speed: int = 0  # 0-250 km/h
    confidence: int = 0  # 0-100 %
    status: int = 0  # 0=发现, 1=确认, 2=认领

    def __post_init__(self):
        if self.type not in VALID_TYPES:
            raise ValueError(f"无效消息类型: {self.type!r}，应为 {VALID_TYPES}")
        if not 0 <= self.target_id <= 255:
            raise ValueError(f"target_id 超范围: {self.target_id}")
        if self.status not in VALID_STATUSES:
            raise ValueError(f"无效状态: {self.status}，应为 {VALID_STATUSES}")


def encode_targets(targets: list[TargetInfo]) -> str:
    """编码目标列表为 ASCII 字符串（≤50 bytes）。

    格式: "T:lat,lon;D:lat,lon"  分号分隔，与 baseline 兼容。
    每条 ~18 chars，50 bytes 内可放 2 条。
    """
    parts = []
    for t in targets:
        part = f"{t.type}:{t.lat:.3f},{t.lon:.3f}"
        parts.append(part)

    payload = ";".join(parts)

    if len(payload.encode("utf-8")) > _MAX_PAYLOAD_BYTES:
        raise ValueError(
            f"编码后 {len(payload.encode('utf-8'))} bytes 超限 "
            f"({len(targets)} 目标, 内容: {payload!r})"
        )

    return payload


def decode_targets(payload: str) -> list[TargetInfo]:
    """解码 ASCII 字符串为目标列表。"""
    if not payload:
        return []

    targets = []
    for part in payload.split(";"):
        part = part.strip()
        if not part or ":" not in part:
            continue
        try:
            # "T:27.000,125.000"
            t_type, coords = part.split(":", 1)
            if len(t_type) != 1 or t_type not in VALID_TYPES:
                continue
            lat_str, lon_str = coords.split(",", 1)
            targets.append(
                TargetInfo(
                    type=t_type,
                    target_id=0,
                    lat=float(lat_str),
                    lon=float(lon_str),
                )
            )
        except (ValueError, IndexError):
            continue

    return targets


# ── 便捷函数（与 baseline 格式兼容）────────────────────────────────────


def encode_announce(lat: float, lon: float) -> str:
    """编码 announce 消息："A:lat,lon"（确认真目标，需要僚机）。"""
    return f"A:{lat:.3f},{lon:.3f}"


def decode_announce(payload: str):
    """解码 announce 消息，返回 (lat, lon) 或 None。"""
    if payload.startswith("A:"):
        try:
            lat_str, lon_str = payload[2:].split(",", 1)
            return float(lat_str), float(lon_str)
        except (ValueError, IndexError):
            pass
    return None


def parse_payload(payload: str):
    """解析通用 payload，返回消息列表。

    每条消息为 (type_char, lat, lon)：
      'T' = tracking 位置
      'A' = announce 确认目标
      'D' = decoy 标记
      'J' = 干扰区
    """
    messages = []
    for part in payload.split(";"):
        part = part.strip()
        if not part or ":" not in part:
            continue
        try:
            t_type, coords = part.split(":", 1)
            if len(t_type) != 1 or t_type not in VALID_TYPES:
                continue
            lat_str, lon_str = coords.split(",", 1)
            messages.append((t_type, float(lat_str), float(lon_str)))
        except (ValueError, IndexError):
            continue
    return messages
