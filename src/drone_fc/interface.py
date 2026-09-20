"""Flight-controller abstract API. Applications depend on this module, not pymavlink."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable, Sequence

from drone_fc.types import AutopilotType, FlightMode, MissionItem, Telemetry


class FlightController(ABC):
    """Unified civil flight-controller API.

    Lifecycle: ``connect`` (opens the link and waits for the autopilot) →
    control / telemetry → ``disconnect``.

    Not thread-safe: one thread per connection. Do not share an instance with
    camera or inference threads.

    实现类内部注释保持中文；本接口文档面向英文客户。
    """

    @abstractmethod
    def connect(self, timeout_s: float = 15.0) -> AutopilotType:
        """Open the vehicle link and wait until the autopilot heartbeat is seen.

        Direct connection: pass a serial or UDP URI in ``ConnectionConfig``.
        No extra sleep is required after this returns.

        Args:
            timeout_s: Seconds to wait, must be > 0. Default 15. Covers both
                transport setup (TCP) and the first PX4/ArduPilot HEARTBEAT.
        Returns:
            AutopilotType.PX4 or ARDUPILOT.
        Raises:
            InvalidArgumentError: timeout_s <= 0, or the URI string cannot be parsed.
            TransportOpenError: socket or serial port cannot be opened.
            HeartbeatTimeoutError: no flight-controller heartbeat in time (includes
                packet counts for diagnostics).
        Blocking: up to timeout_s seconds.
        """

    @abstractmethod
    def disconnect(self) -> None:
        """Close the transport and clear telemetry.

        Args: none.
        Returns: None. Safe to call more than once.
        Blocking: no (closes the socket promptly).
        """

    @abstractmethod
    def reconnect(self, timeout_s: float = 15.0, uri: str | None = None) -> AutopilotType:
        """Close the transport, open it again, and wait for HEARTBEAT.

        Caller-driven. This library does not reconnect on a background thread.
        Use after ``LinkLostError`` or a dropped socket.

        Args:
            timeout_s: Seconds to wait, must be > 0. Default 15.
            uri: Optional new connection string (e.g. USB re-enumerated from
                ``ttyACM0`` to ``ttyACM1``). ``None`` keeps the previous URI.
        Returns:
            AutopilotType.PX4 or ARDUPILOT.
        Raises:
            InvalidArgumentError: timeout_s <= 0, or uri cannot be parsed.
            TransportOpenError: the socket or serial port cannot be opened.
            HeartbeatTimeoutError: no flight-controller heartbeat in time.
        Blocking: up to timeout_s seconds.
        """

    @abstractmethod
    def wait_heartbeat(self, timeout_s: float = 15.0) -> AutopilotType:
        """Wait for a flight-controller HEARTBEAT if ``connect`` already opened the socket.

        ``connect`` already calls this. Use again only after a link drop.

        Args:
            timeout_s: Max wait in seconds, must be > 0. Default 15.
        Returns:
            AutopilotType.PX4 or ARDUPILOT. Ignores camera, OSD, GCS, and
            local-loop heartbeats. Commands target that autopilot sys/comp id.
        Raises:
            NotConnectedError: transport not open.
            HeartbeatTimeoutError: no autopilot heartbeat in time.
        Blocking: up to timeout_s seconds. Sends a local heartbeat at about 1 Hz.
        """

    @abstractmethod
    def telemetry(self) -> Telemetry:
        """Non-blocking snapshot after draining the receive buffer.

        Args: none.
        Returns:
            Telemetry. Missing fields are None; unknown mode is UNKNOWN.
        Raises:
            NotConnectedError: not connected.
            LinkLostError: FC HEARTBEAT older than about 3 s; call ``reconnect``.
        Blocking: no. May send a local heartbeat if >= 1 s since the last one.
        """

    @abstractmethod
    def wait_telemetry(
        self,
        predicate: Callable[[Telemetry], bool],
        timeout_s: float,
    ) -> Telemetry:
        """Block on MAVLink recv until ``predicate`` is true.

        Uses socket timeouts, not ``time.sleep``. Timeout is a failure: it
        does not return a snapshot that failed the predicate.

        Args:
            predicate: Called with each snapshot; return True to stop.
            timeout_s: Max wait in seconds, must be > 0.
        Returns:
            Telemetry that satisfied ``predicate``.
        Raises:
            InvalidArgumentError: timeout_s <= 0.
            NotConnectedError: not connected.
            LinkLostError: FC HEARTBEAT older than about 3 s; call ``reconnect``.
            TelemetryTimeoutError: predicate still false after timeout_s.
                ``telemetry`` on the exception is the last snapshot.
        Blocking: up to timeout_s seconds.
        """

    @abstractmethod
    def spin(self, duration_s: float) -> Telemetry:
        """Pump MAVLink for ``duration_s`` seconds (keeps the 1 Hz heartbeat).

        Use after takeoff when you want to stay in the air for a known time.
        This is not ``time.sleep``: the thread blocks on recv.

        Args:
            duration_s: Duration in seconds, must be > 0.
        Returns:
            Last Telemetry.
        Raises:
            InvalidArgumentError: duration_s <= 0.
            NotConnectedError: not connected.
            LinkLostError: FC HEARTBEAT older than about 3 s; call ``reconnect``.
        Blocking: about duration_s seconds.
        """

    @abstractmethod
    def arm(self) -> None:
        """Arm motors.

        Preconditions: ``connect`` succeeded. Will not arm without a heartbeat.
        Args: none.
        Returns: None. Success means COMMAND_ACK ACCEPTED.
        Raises:
            NotConnectedError: not linked.
            ArmDeniedError: denied or timed out. Check pre-arm in QGC;
                this library never force-arms. Recent STATUSTEXT is attached
                when the vehicle sent any.
        Blocking: waits for COMMAND_ACK, a few seconds.
        """

    @abstractmethod
    def disarm(self) -> None:
        """Disarm motors.

        Args: none.
        Returns: None. In-air disarm may be rejected (CommandFailedError).
        Blocking: waits for COMMAND_ACK.
        """

    @abstractmethod
    def set_mode(self, mode: FlightMode) -> None:
        """Set a semantic flight mode.

        Args:
            mode: Any FlightMode except UNKNOWN.
        Returns: None.
        Raises:
            UnsupportedModeError: UNKNOWN, or no mapping for this firmware/airframe.
            UnsupportedCapabilityError: this airframe cannot change mode.
            CommandFailedError: SET_MODE failed.
        Blocking: waits for mode echo or ACK.
        """

    @abstractmethod
    def takeoff(self, altitude_m: float) -> None:
        """Arm if needed and command takeoff.

        Multicopter: climb to relative-home altitude. Fixed-wing: firmware
        takeoff (runway / hand-launch), not the same physical manoeuvre.

        Args:
            altitude_m: Target relative altitude in meters, (0, 100] on
                multicopter. Plane/VTOL limits are airframe-specific.
        Returns: None when the command is accepted, not when altitude is reached.
            Use ``wait_telemetry`` to wait for height.
        Raises:
            UnsupportedCapabilityError: this airframe cannot take off this way.
            InvalidArgumentError: altitude out of range.
            GpsNotFixedError: no 3D fix and skip_gps_check is False.
            ArmDeniedError / CommandFailedError: arm or takeoff failed.
        Blocking: waits for takeoff ACK, not for climb complete.
        """

    @abstractmethod
    def goto(self, lat: float, lon: float, alt_m: float) -> None:
        """Go to a global point (relative altitude).

        Args:
            lat: Latitude in degrees, [-90, 90].
            lon: Longitude in degrees, [-180, 180].
            alt_m: Relative-home altitude in meters, (0, 500].
        Returns: None when accepted, not when arrived. Use ``wait_telemetry``.
        Raises:
            UnsupportedCapabilityError: this airframe cannot goto.
            InvalidArgumentError: coordinate or altitude out of range.
            CommandFailedError: DO_REPOSITION and the position setpoint both failed.
        Algorithm:
            PX4 sends MAV_CMD_DO_REPOSITION first; on UNSUPPORTED, GUIDED plus
            SET_POSITION_TARGET_GLOBAL_INT. ArduCopter uses GUIDED + setpoint.
            Lat/lon are sent as degE7 (degrees × 1e7).
        Blocking: waits for ACK or setpoint send.
        """

    @abstractmethod
    def land(self) -> None:
        """Land at the current position.

        Args: none.
        Returns: None when accepted. Use ``wait_telemetry`` for touchdown.
        Raises: UnsupportedCapabilityError or CommandFailedError.
        Blocking: waits for ACK, not for ground contact.
        """

    @abstractmethod
    def rtl(self) -> None:
        """Return to launch and land (vehicle RTL).

        Args: none.
        Returns: None.
        Raises: UnsupportedCapabilityError or CommandFailedError.
        Blocking: waits for ACK, not for RTL complete.
        """

    @abstractmethod
    def upload_mission(self, items: Sequence[MissionItem], timeout_s: float = 15.0) -> None:
        """Upload a waypoint list (MAVLink Mission Protocol). Replaces the onboard mission.

        Args:
            items: 1 to 128 MissionItem values. Lat/lon degrees, altitude metres
                above home (0, 500].
            timeout_s: Seconds to finish the request/item handshake, must be > 0.
        Returns: None when MISSION_ACK ACCEPTED.
        Raises:
            UnsupportedCapabilityError: this airframe cannot fly missions.
            InvalidArgumentError: empty list, too many items, or a point is out of range.
            CommandFailedError: protocol NAK or timeout.
        Blocking: up to timeout_s seconds.
        """

    @abstractmethod
    def start_mission(self) -> None:
        """Switch to MISSION mode and start the uploaded mission.

        Args: none.
        Returns: None when the mode or MISSION_START is accepted.
        Raises: UnsupportedCapabilityError or CommandFailedError.
        Blocking: waits for mode echo or ACK.
        """

    @abstractmethod
    def pause_mission(self) -> None:
        """Pause an active mission (hold / loiter). Resume with start_mission.

        Args: none.
        Returns: None.
        Raises: UnsupportedCapabilityError or CommandFailedError.
        Blocking: waits for ACK or HOLD mode.
        """

    @abstractmethod
    def clear_mission(self, timeout_s: float = 5.0) -> None:
        """Clear the onboard mission list.

        Args:
            timeout_s: Seconds to wait for MISSION_ACK, must be > 0.
        Returns: None.
        Raises: UnsupportedCapabilityError, InvalidArgumentError, CommandFailedError.
        Blocking: up to timeout_s seconds.
        """

    @abstractmethod
    def set_position(self, lat: float, lon: float, alt_m: float) -> None:
        """Send one Guided/Offboard position setpoint. Does not wait for arrival.

        PX4 Offboard needs setpoints faster than about 2 Hz: call this in a loop
        with ``spin`` or ``telemetry``. This library does not start a background
        thread. If not already GUIDED/OFFBOARD, one setpoint is sent before the
        mode switch, then another after (PX4 rejects OFFBOARD without a prior
        setpoint stream).

        Args:
            lat: Latitude in degrees, [-90, 90].
            lon: Longitude in degrees, [-180, 180].
            alt_m: Relative-home altitude in meters, (0, 500].
        Returns: None after the setpoint is sent (and GUIDED/OFFBOARD is set).
        Raises: UnsupportedCapabilityError, InvalidArgumentError, CommandFailedError.
        Blocking: may wait for a mode switch; the setpoint itself is not ACKed.
        """

    @abstractmethod
    def set_velocity(self, north_mps: float, east_mps: float, down_mps: float) -> None:
        """Send one Guided/Offboard NED velocity setpoint (metres per second).

        Same streaming rule as ``set_position``. Down positive is MAVLink NED.

        Args:
            north_mps: North velocity, typically [-20, 20].
            east_mps: East velocity, typically [-20, 20].
            down_mps: Down velocity, typically [-20, 20]. Negative is climb.
        Returns: None after the setpoint is sent.
        Raises: UnsupportedCapabilityError, InvalidArgumentError, CommandFailedError.
        Blocking: may wait for a mode switch.
        """

    @abstractmethod
    def set_gimbal(self, pitch_deg: float, yaw_deg: float) -> None:
        """Point the gimbal (Gimbal Protocol v2). Capability is false until a gimbal is seen.

        Args:
            pitch_deg: Pitch in degrees, [-90, 90]. Negative is down.
            yaw_deg: Yaw in degrees, [-180, 180].
        Returns: None when COMMAND_ACK ACCEPTED.
        Raises: UnsupportedCapabilityError, InvalidArgumentError, CommandFailedError.
        Blocking: waits for ACK.
        """

    @abstractmethod
    def center_gimbal(self) -> None:
        """Reset gimbal pitch and yaw to 0 deg.

        Args: none.
        Returns: None.
        Raises: UnsupportedCapabilityError or CommandFailedError.
        Blocking: waits for ACK.
        """
