"""飞控统一接口使用的异常类型。

所有公开方法失败时都应抛出 ``DroneFcError`` 的子类，便于调用方统一捕获。
``command`` / ``result`` 仅在与 MAVLink COMMAND_ACK 相关时有值。
"""

from __future__ import annotations


class DroneFcError(Exception):
    """本库所有异常的基类。"""


class NotConnectedError(DroneFcError):
    """在未调用 connect()、或已经 disconnect() 之后调用了需要链路的方法。"""


class HeartbeatTimeoutError(DroneFcError):
    """在给定时限内没有收到 HEARTBEAT，通常表示 URI、防火墙或 SITL 未启动。"""


class CommandFailedError(DroneFcError):
    """飞控对 COMMAND_LONG / SET_MODE 返回了非 ACCEPTED 的结果，或等待 ACK 超时。"""

    def __init__(self, message: str, command: int | None = None, result: int | None = None) -> None:
        """输入:
            message: 人类可读说明。
            command: MAV_CMD 整型编号，未知时为 None。
            result: MAV_RESULT 整型编号（0=ACCEPTED），超时时为 None。
        """

        super().__init__(message)
        self.command = command
        self.result = result


class ArmDeniedError(DroneFcError):
    """解锁被拒绝：未收到心跳、飞控未就绪，或 COMMAND_ACK 为 DENIED。"""


class GpsNotFixedError(DroneFcError):
    """起飞前要求 3D GPS 定位，但当前 fix_type 不足。SITL 可通过 skip_gps_check 跳过。"""


class UnsupportedModeError(DroneFcError):
    """请求的语义模式无法映射到当前飞控，或遥测得到的模式不可用于 set_mode。"""


class InvalidArgumentError(DroneFcError):
    """调用参数超出文档约定的类型、单位或数值范围。"""
