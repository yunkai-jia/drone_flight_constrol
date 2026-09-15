# 民用飞控统一 Python 接口（MAVLink）

用一套 Python API 同时对接 **PX4** 和 **ArduPilot**。底层只走 MAVLink（`pymavlink`），应用层不直接区分两种固件。

当前范围：连接、识别飞控、解锁、改模式、起飞、前往、降落、返航、遥测。不含武器、打击或任务分配。

分层目标（飞控 / 机载计算机 / 仿真器 / 推理 / 通信 / 集群）与图中库清单见 [docs/需求文档.md](docs/需求文档.md)。

## 架构

```
应用 / 示例脚本
    └── FlightController（抽象接口）
            └── MavlinkFlightController（pymavlink）
                    ├── URI：udp / tcp / serial
                    └── 模式映射：PX4 custom_mode ↔ ArduPilot 多旋翼模式号
```

心跳里的 `HEARTBEAT.autopilot` 用来识别固件：

| 取值 | 固件 |
|------|------|
| 12 `MAV_AUTOPILOT_PX4` | PX4 |
| 3 `MAV_AUTOPILOT_ARDUPILOTMEGA` | ArduPilot |

应用只使用语义模式：`MANUAL`、`HOLD`、`MISSION`、`RTL`、`LAND`、`TAKEOFF`、`GUIDED`。内部再翻译成各飞控的 `SET_MODE`。

PX4 把主/子模式打进 32 位 `custom_mode`（小端）：

```
custom_mode = (sub_mode << 24) | (main_mode << 16)
```

ArduPilot 多旋翼则直接使用整数模式号（如 GUIDED=4，LOITER=5）。映射表针对多旋翼，不含固定翼/车。

## 目录

```
src/drone_fc/
  interface.py              抽象接口（应用应依赖这里）
  types.py                  位置 / 姿态 / 电池 / 遥测 / 连接配置
  exceptions.py             统一异常
  mavlink/connection.py     解析 udp: tcp: serial: URI
  mavlink/modes.py          PX4 ↔ ArduPilot 模式互转
  mavlink/controller.py     统一实现
examples/
  mock_px4_mavlink.py       协议级 PX4 模拟端（无 Gazebo）
  check_connection.py       连通检查
  sitl_takeoff_land.py      起飞 → 悬停 → 降落
  live_telemetry_plot.py    2D 轨迹 + 高度曲线
tests/
  test_modes.py             模式映射单测
  test_connection.py        URI 解析单测
```

## 环境

- Python 3.10+（推荐 3.13 已验证）
- 依赖见 `requirements.txt`：`pymavlink`、`matplotlib`

```bash
pip install -r requirements.txt
```

所有命令在仓库根目录执行，并把 `src` 加到模块搜索路径：

```bash
export PYTHONPATH=src
```

## 快速开始（协议模拟，无需官方 SITL）

本机若没有 PX4 / Gazebo，用协议模拟端发心跳和位置即可验证链路。

**一条命令看图（推荐）：**

```bash
PYTHONPATH=src python examples/live_telemetry_plot.py
```

会自动拉起模拟端，执行 起飞 → 东移 25 m → 北移 15 m → 降落，弹出窗口，并保存 `examples/last_telemetry_plot.png`。无显示器时加 `--no-show`。

**只检查连通（两个终端）：**

```bash
PYTHONPATH=src python examples/mock_px4_mavlink.py
PYTHONPATH=src python examples/check_connection.py
```

预期：打印 `连通成功`，固件 `px4`，GPS `FIX_3D`。

**起飞降落（需先开模拟端）：**

```bash
PYTHONPATH=src python examples/sitl_takeoff_land.py --hover-s 2
```

模拟端默认往 `127.0.0.1:14540` 发 UDP，客户端默认监听同一地址（与 PX4 SITL 的 GCS 端口惯例一致）。

## 对接官方仿真 / 真机

连接串写在 `ConnectionConfig.uri`：

| URI | 含义 |
|-----|------|
| `udp:127.0.0.1:14540` | UDP 监听（PX4 SITL 常见） |
| `tcp:127.0.0.1:5760` | TCP 客户端（ArduPilot SITL 常见） |
| `serial:/dev/ttyACM0:57600` | USB 串口 |
| `udpin:` / `udpout:` | 显式指定 UDP 方向 |

真机起飞前默认要求 3D GPS（`fix_type >= 3`）。仅仿真可设 `skip_gps_check=True`。

```python
from drone_fc import ConnectionConfig, MavlinkFlightController

fc = MavlinkFlightController(
    ConnectionConfig("udp:127.0.0.1:14540", skip_gps_check=True)
)
fc.connect()
print(fc.wait_heartbeat())   # AutopilotType.PX4 或 ARDUPILOT
fc.takeoff(5.0)              # 相对 Home 高度，米，范围 (0, 100]
# fc.goto(lat, lon, alt_m)   # 纬度[-90,90]，经度[-180,180]，高度 (0, 500]
snap = fc.telemetry()
fc.land()
fc.disconnect()
```

`takeoff` / `goto` / `land` 在命令被飞控 ACK 后即返回，**不保证**已经爬到高度或接地。需要到位时请循环读 `telemetry()`，示例见 `live_telemetry_plot.py`。

## 坐标与单位

- 经纬度：WGS84，单位度
- 高度：相对 Home 的米（对应 `GLOBAL_POSITION_INT.relative_alt`，不是椭球高）
- 姿态：度（库内把 MAVLink 弧度乘 `180/π`）
- 电池电压：伏特；电流：安培

可视化里把经纬度换成相对 Home 的东/北米：

```
北向 = (lat - home_lat) × 111320
东向 = (lon - home_lon) × 111320 × cos(home_lat)
```

## 测试

不依赖飞控硬件：

```bash
PYTHONPATH=src python -m unittest discover -s tests -v
```

## 接口一览

`FlightController` 公开方法：

| 方法 | 作用 |
|------|------|
| `connect(timeout_s)` | 打开传输层 |
| `wait_heartbeat(timeout_s)` | 等心跳并识别固件 |
| `telemetry()` | 非阻塞读最新位置/姿态/电量/模式 |
| `arm()` / `disarm()` | 解锁 / 上锁 |
| `set_mode(mode)` | 切换语义模式 |
| `takeoff(altitude_m)` | 起飞 |
| `goto(lat, lon, alt_m)` | 前往一点 |
| `land()` / `rtl()` | 降落 / 返航 |
| `disconnect()` | 关闭连接 |

失败抛 `drone_fc` 下的异常（如 `HeartbeatTimeoutError`、`ArmDeniedError`、`GpsNotFixedError`），详见 `src/drone_fc/exceptions.py`。
