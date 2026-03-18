"""
Gripper Controller for Dynamixel XM430-W350-T via U2D2 (TTL, Protocol 2.0)

Dependencies:
    pip install dynamixel-sdk

Control Table (XM430-W350-T):
    Address 11  | 1B | Operating Mode     (3 = Position Control)
    Address 64  | 1B | Torque Enable
    Address 108 | 4B | Profile Acceleration  (unit: 214.577 rev/min²)
    Address 112 | 4B | Profile Velocity      (unit: 0.229 rpm)
    Address 116 | 4B | Goal Position         (unit: 1 tick, 4096 ticks/rev)
    Address 122 | 1B | Moving                (1 = in motion, 0 = stopped)
    Address 132 | 4B | Present Position      (unit: 1 tick)
"""

import time
import dynamixel_sdk as dxl

# ──────────────────────────────────────────────
# Hardware Configuration  (edit to match your setup)
# ──────────────────────────────────────────────
DEVICE_PORT  = '/dev/ttyUSB0'   # Linux. Windows: 'COM3', macOS: '/dev/tty.usbserial-*'
BAUD_RATE    = 1000000
DXL_ID       = 0
PROTOCOL_VER = 2.0

# ──────────────────────────────────────────────
# Control Table Addresses  (XM430-W350-T, Protocol 2.0)
# ──────────────────────────────────────────────
ADDR_OPERATING_MODE  = 11
ADDR_TORQUE_ENABLE   = 64
ADDR_PROFILE_ACCEL   = 108
ADDR_PROFILE_VEL     = 112
ADDR_GOAL_POSITION   = 116
ADDR_MOVING          = 122
ADDR_PRESENT_POSITION = 132

# ── Add these new constants (alongside existing ones) ──────────────────────

ADDR_GOAL_CURRENT        = 102   # 2 bytes; only active in mode 5
CURRENT_BASED_POS_MODE   = 5     # replaces POSITION_MODE (3) for torque control
MAX_CURRENT_UNITS        = 1193  # default Current Limit (addr 38); unit ≈ 2.69 mA
                                 # 1193 units ≈ motor's rated peak current at 12V

# ──────────────────────────────────────────────
# Motor Constants
# ──────────────────────────────────────────────
TORQUE_ENABLE          = 1
TORQUE_DISABLE         = 0
CURRENT_BASED_POS_MODE = 5        # Operating Mode 5: Current-Based Position Control
POS_RESOLUTION         = 4096     # Ticks per revolution (0.088 deg/tick)
MAX_PROFILE_VEL        = 200      # ~45.8 rpm @ 0.229 rpm/unit
MAX_CURRENT_UNITS      = 1193     # Default Current Limit; 1 unit ≈ 2.69 mA
POSITION_TOLERANCE     = 15       # Ticks; generous for current-based mode
MOTION_TIMEOUT_S       = 10.0
MOVING_ARM_TIMEOUT_S   = 0.25     # Max time to wait for MOVING to go 1 after goal write
POLL_INTERVAL_S        = 0.02


# ─────────────────────────────────────────────
# Internal helpers
# ─────────────────────────────────────────────

def _degrees_to_ticks(degrees: float) -> int:
    """Convert an angle in degrees to Dynamixel position ticks."""
    return int(round(degrees * POS_RESOLUTION / 360.0))


def _ticks_to_degrees(ticks: int) -> float:
    """Convert Dynamixel position ticks back to degrees."""
    return ticks * 360.0 / POS_RESOLUTION


def _check_comm(result: int, error: int, packet_handler, label: str) -> bool:
    """
    Validate a Tx/Rx result. Prints a descriptive message on failure.
    Returns True if communication was successful, False otherwise.
    """
    if result != dxl.COMM_SUCCESS:
        print(f"[ERROR] {label}: {packet_handler.getTxRxResult(result)}")
        return False
    if error != 0:
        print(f"[WARN]  {label} hardware error: {packet_handler.getRxPacketError(error)}")
        # Hardware error is non-fatal for motion; return True but flag it.
    return True

def _wait_for_motion_complete(port_handler, packet_handler, dxl_id: int, goal_ticks: int) -> bool:
    """
    Two-phase motion wait to avoid the MOVING-flag race condition.

    Phase 1 — Arm wait:
        After writing Goal Position, the motor needs a few ms to load its
        trajectory profile. During this window MOVING = 0 (false idle).
        We wait up to MOVING_ARM_TIMEOUT_S for MOVING to go HIGH (= 1),
        indicating the trajectory has actually started.

    Phase 2 — Completion wait:
        Once MOVING is confirmed HIGH (or we timed out phase 1 and the
        motor is a short move that finished before we could catch it),
        we wait for MOVING to go LOW again.

    Returns True if motion completed, False on comms error or timeout.
    """
    # ── Phase 1: wait for MOVING → 1 ──────────────────────────────────────
    arm_deadline = time.time() + MOVING_ARM_TIMEOUT_S
    while time.time() < arm_deadline:
        moving, res, err = packet_handler.read1ByteTxRx(
            port_handler, dxl_id, ADDR_MOVING
        )
        if not _check_comm(res, err, packet_handler, "Read MOVING (arm)"):
            return False
        if moving == 1:
            break
        time.sleep(POLL_INTERVAL_S)
    # Note: if phase 1 times out, the move may be tiny and already done — that's fine.

    # ── Phase 2: wait for MOVING → 0 ──────────────────────────────────────
    deadline = time.time() + MOTION_TIMEOUT_S
    while time.time() < deadline:
        moving, res, err = packet_handler.read1ByteTxRx(
            port_handler, dxl_id, ADDR_MOVING
        )
        if not _check_comm(res, err, packet_handler, "Read MOVING (complete)"):
            return False
        if moving == 0:
            return True
        time.sleep(POLL_INTERVAL_S)

    print(f"[ERROR] Motion timeout ({MOTION_TIMEOUT_S}s) exceeded.")
    return False


# ─────────────────────────────────────────────
# Main gripper control function
# ─────────────────────────────────────────────

# def control_gripper(
#     angle: float,
#     speed: float = 1.0,
#     open_angle: float = 10.0,
#     close_angle: float = 100.0,
#     port: str = DEVICE_PORT,
#     baud: int = BAUD_RATE,
#     dxl_id: int = DXL_ID,
# ) -> int:
def control_gripper(
    angle: float,
    speed: float  = 1.0,
    torque: float = 1.0,          # NEW — fraction of max current [0.0, 1.0]
    open_angle: float  = 285.0,
    close_angle: float = 166.0,
    port: str  = DEVICE_PORT,
    baud: int  = BAUD_RATE,
    dxl_id: int = DXL_ID,
) -> int:
    """
    ...
    Args:
        torque : Fraction of max current/torque [0.0, 1.0] (default 1.0 = full).
                 Maps to Goal Current (addr 102) in Current-based Position Mode.
                 Useful for compliant grasps — e.g. torque=0.3 for soft objects.
    ...
    """
    """
    Move the Dynamixel XM430-W350-T gripper to a target angle.

    Args:
        angle       : Desired gripper angle in degrees.
                      Will be clamped to [open_angle, close_angle].
        speed       : Fractional speed, range [0.0, 1.0].
                      1.0 = ~46 rpm (motor max); 0.5 = ~23 rpm.
        open_angle  : Angle (°) corresponding to fully open  (default: 10°).
        close_angle : Angle (°) corresponding to fully closed (default: 100°).
        port        : Serial port of the U2D2 (default: '/dev/ttyUSB0').
        baud        : Baud rate (default: 57600).
        dxl_id      : Dynamixel servo ID (default: 1).

    Returns:
        1  — Motion completed successfully and position verified.
        0  — Failure at any stage (comm error, timeout, position mismatch).

    Notes:
        • Operating Mode is written once at the start (EEPROM write).
          Avoid calling this function in a tight loop if you can keep a
          persistent connection; use the GripperController class below instead.
        • Requires: pip install dynamixel-sdk
    """

    # ── Clamp and validate inputs ──────────────────────────────────────────
    lo = min(open_angle, close_angle)
    hi = max(open_angle, close_angle)
    angle  = max(lo, min(hi, angle))
    speed  = max(0.01, min(1.0, speed))

    goal_ticks  = _degrees_to_ticks(angle)
    profile_vel = max(1, int(round(speed * MAX_PROFILE_VEL)))

    print(f"[INFO]  Target: {angle:.2f}° → {goal_ticks} ticks | "
          f"Speed: {speed:.2f} ({profile_vel} units ≈ {profile_vel * 0.229:.1f} rpm)")

    # ── Open port ──────────────────────────────────────────────────────────
    port_handler   = dxl.PortHandler(port)
    packet_handler = dxl.PacketHandler(PROTOCOL_VER)

    try:
        if not port_handler.openPort():
            print(f"[ERROR] Cannot open port: {port}")
            return 0

        if not port_handler.setBaudRate(baud):
            print(f"[ERROR] Cannot set baud rate: {baud}")
            port_handler.closePort()
            return 0

        # ── Torque OFF (required before changing Operating Mode) ───────────
        res, err = packet_handler.write1ByteTxRx(
            port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
        )
        if not _check_comm(res, err, packet_handler, "Torque disable"):
            port_handler.closePort()
            return 0

        # ── Set Position Control Mode ──────────────────────────────────────
        # NOTE: This writes to EEPROM. If the motor is already in mode 3,
        # you can skip this write to preserve EEPROM lifespan.
        # res, err = packet_handler.write1ByteTxRx(
        #     port_handler, dxl_id, ADDR_OPERATING_MODE, POSITION_MODE
        # )
        # if not _check_comm(res, err, packet_handler, "Set operating mode"):
        #     port_handler.closePort()
        #     return 0

        res, err = packet_handler.write1ByteTxRx(
            port_handler, dxl_id, ADDR_OPERATING_MODE, CURRENT_BASED_POS_MODE  # ← was POSITION_MODE
        )
        if not _check_comm(res, err, packet_handler, "Set operating mode"):
            port_handler.closePort()
            return 0

        # ── Set Profile Velocity ────────────────────────────────────────────
        res, err = packet_handler.write4ByteTxRx(
            port_handler, dxl_id, ADDR_PROFILE_VEL, profile_vel
        )
        if not _check_comm(res, err, packet_handler, "Set profile velocity"):
            port_handler.closePort()
            return 0

        # ── Torque ON ──────────────────────────────────────────────────────
        res, err = packet_handler.write1ByteTxRx(
            port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
        )
        if not _check_comm(res, err, packet_handler, "Torque enable"):
            port_handler.closePort()
            return 0
        
        # ADD: write Goal Current AFTER torque enable, BEFORE goal position
        goal_current = max(1, int(round(torque * MAX_CURRENT_UNITS)))
        goal_current = min(goal_current, MAX_CURRENT_UNITS)  # never exceed Current Limit

        res, err = packet_handler.write2ByteTxRx(             # ← 2-byte write
            port_handler, dxl_id, ADDR_GOAL_CURRENT, goal_current
        )
        if not _check_comm(res, err, packet_handler, "Set goal current"):
            port_handler.closePort()
            return 0

        print(f"[INFO]  Torque limit: {torque:.2f} → {goal_current} units "
            f"(≈ {goal_current * 2.69:.0f} mA)")

        # ── Write Goal Position ────────────────────────────────────────────
        res, err = packet_handler.write4ByteTxRx(
            port_handler, dxl_id, ADDR_GOAL_POSITION, goal_ticks
        )
        if not _check_comm(res, err, packet_handler, "Write goal position"):
            port_handler.closePort()
            return 0

        # ── Wait for motion to complete ────────────────────────────────────
        deadline = time.time() + MOTION_TIMEOUT_S
        while time.time() < deadline:
            moving, res, err = packet_handler.read1ByteTxRx(
                port_handler, dxl_id, ADDR_MOVING
            )
            if not _check_comm(res, err, packet_handler, "Read MOVING flag"):
                port_handler.closePort()
                return 0
            if moving == 0:
                break
            time.sleep(POLL_INTERVAL_S)
        else:
            print(f"[ERROR] Motion timeout ({MOTION_TIMEOUT_S}s) exceeded.")
            port_handler.closePort()
            return 0

        # ── Verify final position ──────────────────────────────────────────
        # present_ticks, res, err = packet_handler.read4ByteTxRx(
        #     port_handler, dxl_id, ADDR_PRESENT_POSITION
        # )
        # if not _check_comm(res, err, packet_handler, "Read present position"):
        #     port_handler.closePort()
        #     return 0
        if not _wait_for_motion_complete(port_handler, packet_handler, dxl_id, goal_ticks):
            port_handler.closePort()
            return 0

        present_ticks, res, err = packet_handler.read4ByteTxRx(
            port_handler, dxl_id, ADDR_PRESENT_POSITION
        )
        if not _check_comm(res, err, packet_handler, "Read present position"):
            port_handler.closePort()
            return 0

        # Mask to 32-bit unsigned (SDK may return signed on some platforms)
        present_ticks = present_ticks & 0xFFFFFFFF
        tick_error = abs(present_ticks - goal_ticks)

        if tick_error > POSITION_TOLERANCE:
            print(f"[ERROR] Position mismatch: goal={goal_ticks} ticks "
                  f"({angle:.2f}°), present={present_ticks} ticks "
                  f"({_ticks_to_degrees(present_ticks):.2f}°), "
                  f"error={tick_error} ticks.")
            port_handler.closePort()
            return 0

        print(f"[OK]    Reached {_ticks_to_degrees(present_ticks):.2f}° "
              f"(error: {tick_error} ticks = {tick_error * 360/POS_RESOLUTION:.3f}°)")
        port_handler.closePort()
        return 1

    except Exception as exc:
        print(f"[EXCEPTION] {exc}")
        try:
            port_handler.closePort()
        except Exception:
            pass
        return 0


# ─────────────────────────────────────────────
# Class-based controller for persistent sessions
# (recommended for repeated calls in a robot loop)
# ─────────────────────────────────────────────

class GripperController:
    """
    Persistent-connection gripper controller with torque (current) control.

    Uses Current-Based Position Mode (mode 5) which simultaneously controls
    position trajectory AND limits the maximum current, enabling compliant grasps.

    Usage:
        with GripperController(open_angle=285.0, close_angle=166.0) as g:
            g.open(speed=0.8)
            g.move(240.0, speed=0.5, torque=0.4)
            g.close(speed=1.0, torque=0.6)
    """

    def __init__(
        self,
        port: str   = DEVICE_PORT,
        baud: int   = BAUD_RATE,
        dxl_id: int = DXL_ID,
        open_angle:  float = 285.0,
        close_angle: float = 166.0,
        default_speed:  float = 1.0,   # used by open() / close() if not overridden
        default_torque: float = 1.0,   # used by open() / close() if not overridden
    ):
        self.port           = port
        self.baud           = baud
        self.dxl_id         = dxl_id
        self.open_angle     = open_angle
        self.close_angle    = close_angle
        self.default_speed  = max(0.01, min(1.0, default_speed))
        self.default_torque = max(0.01, min(1.0, default_torque))
        self._connected     = False

        self.port_handler   = dxl.PortHandler(port)
        self.packet_handler = dxl.PacketHandler(PROTOCOL_VER)

        self._connect()

    # ── Internal ────────────────────────────────────────────────────────────

    def _connect(self):
        if not self.port_handler.openPort():
            raise IOError(f"Cannot open port: {self.port}")
        if not self.port_handler.setBaudRate(self.baud):
            raise IOError(f"Cannot set baud rate: {self.baud}")

        # One-time EEPROM init: mode → RAM: profile vel + torque on
        for addr, val, nbytes, label in [
            (ADDR_TORQUE_ENABLE,   TORQUE_DISABLE,          1, "torque off"),
            (ADDR_OPERATING_MODE,  CURRENT_BASED_POS_MODE,  1, "current-based pos mode"),
            (ADDR_TORQUE_ENABLE,   TORQUE_ENABLE,            1, "torque on"),
        ]:
            write_fn = (self.packet_handler.write1ByteTxRx if nbytes == 1
                        else self.packet_handler.write4ByteTxRx)
            res, err = write_fn(self.port_handler, self.dxl_id, addr, val)
            if not _check_comm(res, err, self.packet_handler, label):
                raise RuntimeError(f"Init failed at: {label}")

        self._connected = True
        print(f"[INFO]  GripperController connected — {self.port} @ {self.baud} baud "
              f"| ID={self.dxl_id} | mode=Current-Based Position (5)")

    def _write4(self, addr: int, val: int, label: str) -> bool:
        res, err = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.dxl_id, addr, val)
        return _check_comm(res, err, self.packet_handler, label)

    def _write2(self, addr: int, val: int, label: str) -> bool:
        res, err = self.packet_handler.write2ByteTxRx(
            self.port_handler, self.dxl_id, addr, val)
        return _check_comm(res, err, self.packet_handler, label)

    def _write1(self, addr: int, val: int, label: str) -> bool:
        res, err = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.dxl_id, addr, val)
        return _check_comm(res, err, self.packet_handler, label)

    # ── Public API ──────────────────────────────────────────────────────────

    def move(
        self,
        angle: float,
        speed: float  = None,   # None → uses self.default_speed
        torque: float = None,   # None → uses self.default_torque
    ) -> int:
        """
        Move gripper to angle (°).

        Args:
            angle  : Target angle in degrees, clamped to [open_angle, close_angle].
            speed  : Speed fraction [0.01, 1.0]. Defaults to self.default_speed.
            torque : Current limit fraction [0.01, 1.0]. Defaults to self.default_torque.
                     Lower = less force = safer for delicate objects.
                     e.g. torque=0.3 → ~960 mA → ~0.35 Nm holding torque.

        Returns:
            1 on success, 0 on failure.
        """
        if not self._connected:
            print("[ERROR] Not connected.")
            return 0

        speed  = self.default_speed  if speed  is None else max(0.01, min(1.0, speed))
        torque = self.default_torque if torque is None else max(0.01, min(1.0, torque))

        lo = min(self.open_angle, self.close_angle)
        hi = max(self.open_angle, self.close_angle)
        angle = max(lo, min(hi, angle))

        goal_ticks   = _degrees_to_ticks(angle)
        profile_vel  = max(1, int(round(speed  * MAX_PROFILE_VEL)))
        goal_current = max(1, min(MAX_CURRENT_UNITS, int(round(torque * MAX_CURRENT_UNITS))))

        print(f"[INFO]  → {angle:.1f}° ({goal_ticks} ticks) | "
              f"spd={speed:.2f} ({profile_vel} units) | "
              f"τ={torque:.2f} ({goal_current} units ≈ {goal_current*2.69:.0f} mA)")

        try:
            if not self._write4(ADDR_PROFILE_VEL,  profile_vel,  "Set profile velocity"):
                return 0
            if not self._write2(ADDR_GOAL_CURRENT,  goal_current, "Set goal current"):
                return 0
            if not self._write4(ADDR_GOAL_POSITION, goal_ticks,   "Write goal position"):
                return 0

            # ── Two-phase wait (fixes MOVING-flag race condition) ────────────
            if not _wait_for_motion_complete(
                self.port_handler, self.packet_handler, self.dxl_id, goal_ticks
            ):
                return 0

            # ── Verify final position ────────────────────────────────────────
            present_ticks, res, err = self.packet_handler.read4ByteTxRx(
                self.port_handler, self.dxl_id, ADDR_PRESENT_POSITION
            )
            if not _check_comm(res, err, self.packet_handler, "Read position"):
                return 0

            present_ticks = present_ticks & 0xFFFFFFFF
            tick_error    = abs(present_ticks - goal_ticks)

            # In current-based mode the motor may stop slightly before the
            # goal if torque is low and friction is high — widen tolerance
            # or skip the position check if you're using torque for compliance.
            if tick_error > POSITION_TOLERANCE:
                print(f"[WARN]  Position offset: goal={goal_ticks} "
                      f"({angle:.1f}°), present={present_ticks} "
                      f"({_ticks_to_degrees(present_ticks):.1f}°), "
                      f"Δ={tick_error} ticks — may be torque-limited contact.")
                # Still return 1: reaching the torque limit IS a valid grasp event
                return 1

            print(f"[OK]    Reached {_ticks_to_degrees(present_ticks):.1f}° "
                  f"(Δ {tick_error} ticks = {tick_error*360/POS_RESOLUTION:.2f}°)")
            return 1

        except Exception as exc:
            print(f"[EXCEPTION] {exc}")
            return 0

    def open(self, speed: float = None, torque: float = None) -> int:
        """Move to fully open position."""
        return self.move(self.open_angle, speed=speed, torque=torque)

    def close(self, speed: float = None, torque: float = None) -> int:
        """Move to fully closed position."""
        return self.move(self.close_angle, speed=speed, torque=torque)

    def set_defaults(self, speed: float = None, torque: float = None):
        """Update the default speed and/or torque for subsequent calls."""
        if speed  is not None: self.default_speed  = max(0.01, min(1.0, speed))
        if torque is not None: self.default_torque = max(0.01, min(1.0, torque))

    def close_connection(self):
        """Disable torque and release the serial port."""
        if self._connected:
            self._write1(ADDR_TORQUE_ENABLE, TORQUE_DISABLE, "torque off (shutdown)")
            self.port_handler.closePort()
            self._connected = False
            print("[INFO]  GripperController disconnected.")

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close_connection()



# ─────────────────────────────────────────────
# Example usage
# ─────────────────────────────────────────────

if __name__ == "__main__":

    # # ── Option A: Simple one-shot function call ────────────────────────────
    # result = control_gripper(170.0, speed=0.8)
    # print("Result:", result)   # 1 = success, 0 = failure

    # ── Option B: Persistent class (recommended for robot pipelines) ────────
    # with GripperController(open_angle=285.0, close_angle=166.0) as gripper:
    #     gripper.open(speed=0.8)
    #     time.sleep(1)
    #     gripper.move(240.0, speed=0.8)   # mid-grasp
    #     time.sleep(1)
    #     gripper.close(speed=1.0)
    #     time.sleep(1)

    gripper= GripperController(open_angle=285.0, close_angle=166.0, default_speed=0.2, default_torque=0.5)
    for i in range(10):
        gripper.open()
        gripper.move(240.0)   # mid-grasp
        gripper.close()
    gripper.close_connection()