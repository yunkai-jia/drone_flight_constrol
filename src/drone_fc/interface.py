"""飞控抽象接口：应用只依赖本模块，不直接使用 pymavlink。"""

from __future__ import annotations

from abc import ABC, abstractmethod

from drone_fc.types import AutopilotType, FlightMode, Telemetry


class FlightController(ABC):
    """民用飞控统一接口。

    生命周期：``connect`` → ``wait_heartbeat`` → 控制/遥测 → ``disconnect``。
    实现类不是线程安全的，调用方应在同一线程使用。
    """

    @abstractmethod
    def connect(self, timeout_s: float = 10.0) -> None:
        """打开到飞控的传输层连接。

        输入:
            timeout_s: 建连超时，单位秒，必须 > 0。对 UDP 监听几乎立即返回；
                对 TCP 表示连接对端的最长等待。默认 10。
        输出:
            None。成功后后续方法可发送 MAVLink。
        失败:
            InvalidArgumentError: timeout_s <= 0。
            DroneFcError: URI 无法解析或套接字失败。
        阻塞: 最多 timeout_s 秒。
        """

    @abstractmethod
    def disconnect(self) -> None:
        """关闭传输层并清空遥测缓存。

        输入: 无。
        输出: None。重复调用是安全的。
        阻塞: 否（尽快关闭套接字）。
        """

    @abstractmethod
    def wait_heartbeat(self, timeout_s: float = 15.0) -> AutopilotType:
        """等待 HEARTBEAT 并识别固件类型。

        输入:
            timeout_s: 最长等待，单位秒，必须 > 0。默认 15。
        输出:
            AutopilotType.PX4 或 ARDUPILOT；无法归类时为 UNKNOWN。
        失败:
            NotConnectedError: 尚未 connect。
            HeartbeatTimeoutError: 时限内无心跳。
        阻塞: 最多 timeout_s 秒。
        """

    @abstractmethod
    def telemetry(self) -> Telemetry:
        """读取最近一次汇聚的遥测快照（非阻塞排空接收缓冲后返回）。

        输入: 无。
        输出:
            Telemetry。尚未收到的字段为 None；mode 在无法识别时为 UNKNOWN。
        失败:
            NotConnectedError: 尚未 connect。
        阻塞: 否。
        """

    @abstractmethod
    def arm(self) -> None:
        """解锁电机。

        前置: 已收到心跳。不会在未心跳时发送解锁。
        输入: 无。
        输出: None。飞控 ACK 为 ACCEPTED 视为成功。
        失败:
            NotConnectedError / HeartbeatTimeoutError 语义上的未就绪。
            ArmDeniedError: ACK 拒绝或超时。
        阻塞: 等待 COMMAND_ACK，默认数秒。
        """

    @abstractmethod
    def disarm(self) -> None:
        """上锁电机。

        输入: 无。
        输出: None。空中上锁可能被飞控拒绝，此时抛 CommandFailedError。
        阻塞: 等待 COMMAND_ACK。
        """

    @abstractmethod
    def set_mode(self, mode: FlightMode) -> None:
        """切换语义飞行模式。

        输入:
            mode: 除 UNKNOWN 外的 FlightMode。
        输出: None。
        失败:
            UnsupportedModeError: UNKNOWN，或当前固件无法映射。
            CommandFailedError: SET_MODE 失败。
        阻塞: 等待模式回读或 ACK。
        """

    @abstractmethod
    def takeoff(self, altitude_m: float) -> None:
        """解锁（若需要）并爬升到相对 Home 高度。

        输入:
            altitude_m: 目标相对高，单位米，开闭区间 (0, 100]。
        输出: None。命令被接受即返回，不保证已到达高度。
        失败:
            InvalidArgumentError: 高度越界。
            GpsNotFixedError: 未 3D 定位且未设置 skip_gps_check。
            ArmDeniedError / CommandFailedError: 解锁或起飞命令失败。
        阻塞: 等待起飞相关 ACK，不阻塞等待爬升完成。
        """

    @abstractmethod
    def goto(self, lat: float, lon: float, alt_m: float) -> None:
        """前往一个全球坐标点（相对高）。

        输入:
            lat: 纬度，度，[-90, 90]。
            lon: 经度，度，[-180, 180]。
            alt_m: 相对 Home 高度，米，(0, 500]。
        输出: None。命令接受即返回，不保证到达。
        失败:
            InvalidArgumentError: 坐标或高度越界。
            CommandFailedError: DO_REPOSITION 与位置设定点均失败。
        算法:
            PX4 先发 MAV_CMD_DO_REPOSITION；ACK 为 UNSUPPORTED 时再切 GUIDED
            并发送 SET_POSITION_TARGET_GLOBAL_INT。ArduPilot 多旋翼直接切 GUIDED
            后发位置设定点。经纬度按度乘 1e7 转为整数（degE7）。
        阻塞: 等待 ACK 或设定点发送完成。
        """

    @abstractmethod
    def land(self) -> None:
        """在当前位置降落。

        输入: 无。
        输出: None。命令接受即返回。
        失败: CommandFailedError。
        阻塞: 等待 ACK，不等待接地。
        """

    @abstractmethod
    def rtl(self) -> None:
        """返航并在 Home 点降落（飞控 RTL 逻辑）。

        输入: 无。
        输出: None。
        失败: CommandFailedError。
        阻塞: 等待 ACK，不等待返航完成。
        """
