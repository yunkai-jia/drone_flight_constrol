"""将连接 URI 解析为 pymavlink 可使用的设备描述。"""

from __future__ import annotations

from dataclasses import dataclass

from drone_fc.exceptions import InvalidArgumentError


@dataclass(frozen=True)
class ParsedConnection:
    """解析后的连接描述。

    Attributes:
        device: 传给 ``mavutil.mavlink_connection`` 的 device 字符串。
        baud: 串口波特率；非串口时为 None。
    """

    device: str
    baud: int | None = None


_UDP_SCHEMES = ("udp", "udpin", "udpout")


def parse_connection_uri(uri: str) -> ParsedConnection:
    """解析 ``udp:`` / ``tcp:`` / ``serial:`` 连接串。

    输入:
        uri: 非空字符串。空白首尾会被去掉。
            - ``udp:host:port`` → pymavlink ``udpin:host:port``（监听，适配 PX4 SITL）
            - ``udpin:host:port`` / ``udpout:host:port`` 原样保留 scheme
            - ``tcp:host:port`` → ``tcp:host:port``
            - ``serial:device:baud`` → device 为串口路径，baud 为整数
    输出:
        ParsedConnection。
    失败:
        InvalidArgumentError: 空串、缺字段、端口/波特率无法解析或越界。

    端口合法范围 [1, 65535]；波特率合法范围 [1200, 1000000]。
    """

    if not isinstance(uri, str) or not uri.strip():
        raise InvalidArgumentError("uri must be a non-empty string")

    text = uri.strip()
    scheme, sep, rest = text.partition(":")
    if not sep or not rest:
        raise InvalidArgumentError(
            f"cannot parse URI {uri!r}; expected udp:host:port / tcp:host:port / serial:device:baud"
        )
    scheme = scheme.lower()

    if scheme in _UDP_SCHEMES or scheme == "tcp":
        host, port = _split_host_port(rest, uri)
        if scheme == "udp":
            scheme = "udpin"
        return ParsedConnection(device=f"{scheme}:{host}:{port}")

    if scheme == "serial":
        device, baud = _split_serial(rest, uri)
        return ParsedConnection(device=device, baud=baud)

    raise InvalidArgumentError(
        f"unsupported URI scheme {scheme!r}; use udp/udpin/udpout/tcp/serial"
    )


def _split_host_port(rest: str, uri: str) -> tuple[str, int]:
    """从 ``host:port`` 拆出主机与端口。

    输入:
        rest: scheme 之后的部分，例如 ``127.0.0.1:14540``。
        uri: 完整 URI，仅用于错误信息。
    输出:
        (host, port)，port 在 [1, 65535]。
    说明:
        按最后一个冒号分割，避免误伤 IPv6（当前未正式支持 IPv6 字面量）。
    """

    host, sep, port_text = rest.rpartition(":")
    if not sep or not host or not port_text:
        raise InvalidArgumentError(f"URI {uri!r} is missing host:port")
    port = _parse_int(port_text, "port")
    if port < 1 or port > 65535:
        raise InvalidArgumentError(f"port {port} is outside [1, 65535]")
    return host, port


def _split_serial(rest: str, uri: str) -> tuple[str, int]:
    """从右侧拆出波特率，其余为设备名。

    输入:
        rest: ``serial:`` 之后的部分，例如 ``/dev/ttyACM0:57600`` 或 ``COM3:115200``。
        uri: 完整 URI，仅用于错误信息。
    输出:
        (device, baud)，baud 在 [1200, 1000000]。
    """

    device, sep, baud_text = rest.rpartition(":")
    if not sep or not device or not baud_text:
        raise InvalidArgumentError(f"URI {uri!r} must be serial:device:baud")
    baud = _parse_int(baud_text, "baud")
    if baud < 1200 or baud > 1_000_000:
        raise InvalidArgumentError(f"baud {baud} is outside [1200, 1000000]")
    return device, baud


def _parse_int(text: str, label: str) -> int:
    """把十进制字符串转为 int。

    输入:
        text: 不含正负号以外字符的十进制串。
        label: 出错时用的字段名（端口 / 波特率）。
    """

    try:
        return int(text, 10)
    except ValueError as exc:
        raise InvalidArgumentError(f"{label} is not an integer: {text!r}") from exc
