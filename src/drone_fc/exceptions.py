"""Exceptions for the unified flight-controller API.

Public messages are English (customer-facing). All failures raise a
``DroneFcError`` subclass. ``command`` / ``result`` are set only for
COMMAND_ACK failures.
"""

from __future__ import annotations


class DroneFcError(Exception):
    """Base class for every error in this library."""


class NotConnectedError(DroneFcError):
    """Called a link method before ``connect()`` or after ``disconnect()``."""


class TransportOpenError(DroneFcError):
    """Socket or serial port could not be opened (path, permission, or network)."""

    def __init__(
        self,
        message: str,
        uri: str | None = None,
        device: str | None = None,
    ) -> None:
        """Args:
            message: Human-readable English text.
            uri: Original ``ConnectionConfig.uri``, if known.
            device: Parsed transport device string (serial path or ``udp:`` host).
        """

        super().__init__(message)
        self.uri = uri
        self.device = device


class HeartbeatTimeoutError(DroneFcError):
    """No flight-controller HEARTBEAT within the timeout (URI, serial, or vehicle offline)."""

    def __init__(
        self,
        message: str,
        packets_seen: int = 0,
        fc_heartbeats: int = 0,
        other_heartbeats: int = 0,
        last_msg_type: str | None = None,
    ) -> None:
        """Args:
            message: Human-readable English text, including a short hint.
            packets_seen: MAVLink frames received while waiting (including loopback).
            fc_heartbeats: HEARTBEAT frames from PX4/ArduPilot (0 on timeout).
            other_heartbeats: HEARTBEAT from GCS, camera, OSD, or similar.
            last_msg_type: ``get_type()`` of the last frame, or None if none arrived.
        """

        super().__init__(message)
        self.packets_seen = int(packets_seen)
        self.fc_heartbeats = int(fc_heartbeats)
        self.other_heartbeats = int(other_heartbeats)
        self.last_msg_type = last_msg_type


class LinkLostError(DroneFcError):
    """Flight-controller HEARTBEAT went stale after a successful ``connect()``."""

    def __init__(self, message: str, age_s: float | None = None) -> None:
        """Args:
            message: Human-readable English text.
            age_s: Seconds since the last FC HEARTBEAT, if known.
        """

        super().__init__(message)
        self.age_s = age_s


class TelemetryTimeoutError(DroneFcError):
    """``wait_telemetry`` ended without the predicate becoming true."""

    def __init__(
        self,
        message: str,
        timeout_s: float | None = None,
        telemetry: object | None = None,
    ) -> None:
        """Args:
            message: Human-readable English text.
            timeout_s: Wait budget in seconds.
            telemetry: Last snapshot (may still have None fields).
        """

        super().__init__(message)
        self.timeout_s = timeout_s
        self.telemetry = telemetry


class CommandFailedError(DroneFcError):
    """COMMAND_LONG / SET_MODE was not ACCEPTED, or ACK timed out."""

    def __init__(self, message: str, command: int | None = None, result: int | None = None) -> None:
        """Args:
            message: Human-readable English text.
            command: MAV_CMD id, or None if unknown.
            result: MAV_RESULT id (0=ACCEPTED), or None on timeout.
        """

        super().__init__(message)
        self.command = command
        self.result = result


class ArmDeniedError(DroneFcError):
    """Arm rejected: no heartbeat, pre-arm checks, or COMMAND_ACK DENIED.

    Do not force-arm from this library. Inspect the pre-arm list in
    QGroundControl or Mission Planner. ``status_text`` is the recent
    STATUSTEXT buffer when the vehicle sent one; it is not a substitute
    for the GCS pre-arm list.
    """

    def __init__(self, message: str, status_text: str | None = None) -> None:
        """Args:
            message: Human-readable English text.
            status_text: Joined STATUSTEXT lines, or None if none were seen.
        """

        super().__init__(message)
        self.status_text = status_text


class GpsNotFixedError(DroneFcError):
    """Takeoff requires 3D GPS (fix_type >= 3). SITL may set skip_gps_check."""


class UnsupportedModeError(DroneFcError):
    """Semantic mode has no mapping for this firmware and airframe, or UNKNOWN was passed to set_mode."""


class UnsupportedCapabilityError(DroneFcError):
    """The connected vehicle cannot perform this action (airframe or not yet implemented)."""


class InvalidArgumentError(DroneFcError):
    """A call argument is the wrong type, unit, or range."""
