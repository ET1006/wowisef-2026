"""
Gamepad -> Arduino direct USB-serial bridge (laptop side)
========================================================
Variant of prelim-ver/gamepad_to_arduino.py that talks straight to the
Arduino over USB-serial, no Pi in the loop. Implements the full protocol
the current raspi_bridge/arduino/arduino_bridge/arduino_bridge.ino sketch
exposes, which is a superset of what the prelim script supported:

  - CSV drive stream: <speed>,<dir>,<cmd>      (unchanged)
  - All six button-pulse commands:
        up_attach / up_detach / down_attach / down_detach
        both_attach / both_detach              (unchanged from prelim)
  - New:  estop    -- latches the Arduino, requires a button press to recover
  - New:  WARN:<sensor> -- sent automatically when:
            * the in-band TOF thresholds are violated (operator-tunable),
            * OR the script sees the Arduino publish TOF_OFFLINE (0xFFFF)
              for either sensor. This replaces the dashboard's role of
              surfacing offline sensors; the Arduino already has the
              watchdog retry logic, we just need to trigger the latch.

Inbound from the Arduino is parsed into a small state struct:
  STS:{...json...}    periodic telemetry (every 200 ms)
  ACK:<cmd>           per-command echo
  WARNED:<sensor>     per-WARN echo
  INIT: ...           boot banner

Run:
    pip install pygame pyserial
    python3 gamepad_to_arduino_direct.py
    python3 gamepad_to_arduino_direct.py --port /dev/cu.usbmodem14201
    python3 gamepad_to_arduino_direct.py --test   # no Arduino / no controller
    python3 gamepad_to_arduino_direct.py --test --controller   # pad only
"""

import argparse
import json
import re
import sys
import threading
import time
from dataclasses import dataclass, field

import pygame
from pygame._sdl2 import controller
import serial
import serial.tools.list_ports


# ---------------------------- Configuration (preserved) ----------------------------
BAUD_RATE = 115200  # Matches arduino_bridge.ino setup() -> Serial.begin(115200)
SEND_RATE = 25      # 25 Hz drive stream (~40 ms per frame) -- slower than
                       # 30 Hz gives macOS's USB CDC driver enough headroom
                       # to deliver each 22-byte payload in a single USB
                       # transaction. At 30 Hz some writes get split
                       # mid-line, which the ESP's readStringUntil('\n')
                       # then sees as "0,FORW" + "ARD,..." and rejects
                       # the first fragment as malformed CSV.
HOLD_FRAMES = 1     # ~100 ms button-pulse hold (matches prelim-ver script)

# Per-sensor in-band window. The script auto-sends WARN:<sensor> when a TOF
# reading falls outside this window. Defaults mirror mac_dashboard.py so the
# direct-USB script behaves the same as the dashboard with no flags set.
TOF_UP_MIN_MM   = 30
TOF_UP_MAX_MM   = 400
TOF_DOWN_MIN_MM = 30
TOF_DOWN_MAX_MM = 400

# Sentinel the Arduino publishes when a TOF sensor is down / no fresh sample.
# Mirrors VL53L0X's internal timeout value, far above any sensible threshold.
TOF_OFFLINE = 0xFFFF

# All command tokens the current sketch accepts. Anything outside this set
# triggers an "ACK:<cmd>" on the Arduino (the unknown-command echo path)
# which would still confuse downstream logic, so we filter to this set.
VALID_COMMANDS = {
    "NONE",
    "up_attach", "up_detach",
    "down_attach", "down_detach",
    "both_attach", "both_detach",
    "axon_open", "axon_close",
    "estop",
}


# ---------------------------- Helpers (from gamepad_to_arduino.py) ----------------------------
def normalize_stick(val):
    return max(0, min(255, int((val + 32768) / 65535.0 * 255)))

def normalize_trigger(val):
    if val < 0:
        val = (val + 32768) / 65535.0 * 255
    else:
        val = (val / 32767.0) * 255
    return max(0, min(255, int(val)))

def get_21_channels(pad):
    """Reads 21 discrete channels using SDL2 standard numerical IDs."""
    lx = normalize_stick(pad.get_axis(0))
    ly = normalize_stick(pad.get_axis(1))
    rx = normalize_stick(pad.get_axis(2))
    ry = normalize_stick(pad.get_axis(3))
    l2 = normalize_trigger(pad.get_axis(4))
    r2 = normalize_trigger(pad.get_axis(5))

    cross    = 1 if pad.get_button(0)  else 0
    circle   = 1 if pad.get_button(1)  else 0
    square   = 1 if pad.get_button(2)  else 0
    triangle = 1 if pad.get_button(3)  else 0
    ps_btn   = 1 if pad.get_button(5)  else 0

    dp_up    = 1 if pad.get_button(11) else 0
    dp_down  = 1 if pad.get_button(12) else 0
    dp_left  = 1 if pad.get_button(13) else 0
    dp_right = 1 if pad.get_button(14) else 0

    l1       = 1 if pad.get_button(9)  else 0
    r1       = 1 if pad.get_button(10) else 0
    share    = 1 if pad.get_button(4)  else 0
    options  = 1 if pad.get_button(6)  else 0
    l3       = 1 if pad.get_button(7)  else 0
    r3       = 1 if pad.get_button(8)  else 0

    return [
        lx, ly, rx, ry, l2, r2,
        cross, circle, square, triangle, ps_btn,
        dp_up, dp_down, dp_left, dp_right,
        l1, r1, share, options, l3, r3,
    ]


def find_arduino_port():
    """Auto-detect the Arduino serial port on macOS."""
    for port in serial.tools.list_ports.comports():
        if "usbmodem" in port.device or "usbserial" in port.device:
            return port.device
    return None


def _format_tof(value: int, enabled: bool) -> str:
    """Format a TOF sensor reading for display.
    
    Args:
        value: Raw sensor value in millimeters
        enabled: Whether TOF sensors are enabled globally
        
    Returns:
        Formatted string: "123mm", "OFFLINE", or "DISABLED"
    """
    if not enabled:
        return "DISABLED"
    if value == TOF_OFFLINE:
        return "OFFLINE"
    return f"{value}mm"


# ---------------------------- Arduino-side state ----------------------------
@dataclass
class ArduinoState:
    """Snapshot of everything the Arduino has told us recently.

    Populated by the reader thread, read by the main loop and any debug
    prints. `last_received_at` lets us surface 'Arduino offline' to the
    operator even though the drive stream is one-way from our side.
    """
    tof_up: int = 0
    tof_down: int = 0
    tof_enabled: bool = True
    drive_speed: int = 0
    direction: str = "FORWARD"
    bar_pose: str = "parallel"
    last_ack: str = "NONE"
    latched: bool = False
    latch_reason: str = ""
    last_received_at: float = 0.0
    # Edge-trigger log of STS / ACK / WARNED / INIT, capped to keep memory
    # bounded if the Arduino keeps streaming for hours.
    recent_events: list = field(default_factory=list)

    def record_event(self, kind: str, payload: str) -> None:
        self.recent_events.append((time.monotonic(), kind, payload))
        if len(self.recent_events) > 200:
            self.recent_events = self.recent_events[-200:]


# ---------------------------- Inbound line parsing ----------------------------
# arduino_bridge.ino emits four line types. The order matters: STS lines
# start with "STS:" so we can route them straight into JSON; the rest are
# small tagged tokens.
_ACK_RE     = re.compile(r"^ACK:(.+)$")
_WARNED_RE  = re.compile(r"^WARNED:(up|down)$")
_INIT_RE    = re.compile(r"^INIT:(.*)$")

def parse_inbound_line(line: str, state: ArduinoState) -> None:
    """Apply one decoded Arduino line to the shared state object."""
    line = line.strip()
    if not line:
        return
    state.last_received_at = time.monotonic()
    if line.startswith("STS:"):
        # Tolerate partial frames -- the Arduino occasionally splits a write
        # across two reads when the Python side is faster than the chip's
        # 200 ms cadence. json.loads raises, which we just log.
        try:
            obj = json.loads(line[4:])
        except json.JSONDecodeError as e:
            print(f"\n[ARDUINO REPLY] malformed STS: {e} :: {line!r}")
            return
        tof = obj.get("tof") or {}
        state.tof_up       = int(tof.get("up",   state.tof_up))
        state.tof_down     = int(tof.get("down", state.tof_down))
        state.tof_enabled  = bool(tof.get("enabled", True))
        state.drive_speed  = int(obj.get("speed", state.drive_speed))
        state.direction    = str(obj.get("dir",   state.direction))
        state.bar_pose     = str(obj.get("pose",  state.bar_pose))
        state.last_ack     = str(obj.get("ack",   state.last_ack))
        state.latched      = bool(obj.get("latched", state.latched))
        state.latch_reason = str(obj.get("latch_reason", state.latch_reason))
        state.record_event("STS", "")
    elif _ACK_RE.match(line):
        cmd = _ACK_RE.match(line).group(1)
        state.last_ack = cmd
        state.record_event("ACK", cmd)
        print(f"\n[ARDUINO ACK] {cmd}")
    elif _WARNED_RE.match(line):
        sensor = _WARNED_RE.match(line).group(1)
        state.record_event("WARNED", sensor)
        print(f"\n[ARDUINO] WARNED:{sensor} (latch engaged on Arduino)")
    elif _INIT_RE.match(line):
        rest = _INIT_RE.match(line).group(1).strip()
        state.record_event("INIT", rest)
        print(f"\n[ARDUINO BOOT] {rest}")
    else:
        # Anything else: print but don't error. The sketch may grow new
        # diagnostic prefixes; we want them visible in the operator log.
        state.record_event("RAW", line)
        print(f"\n[ARDUINO REPLY] {line}")


# ---------------------------- Serial reader thread ----------------------------
class SerialReader(threading.Thread):
    """Decode lines from the Arduino off the main tick.

    Daemon thread; cleanly exited when the program terminates because the
    underlying serial.Serial is closed and readline() raises. `state` is
    a shared ArduinoState mutated under the GIL -- integers and small
    strings are atomic in CPython, so we don't bother with an explicit
    lock. If you ever extend this to multi-field tuples, add a lock.
    """
    def __init__(self, ser: serial.Serial, state: ArduinoState):
        super().__init__(daemon=True)
        self._ser = ser
        self._state = state
        self._stop = threading.Event()
        self._line_count = 0

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        while not self._stop.is_set():
            try:
                # Blocking readline is fine here because the thread is
                # dedicated. timeout=0.5 lets us notice the stop flag.
                raw = self._ser.readline()
            except (serial.SerialException, OSError) as e:
                print(f"\n[serial] read error: {e}", file=sys.stderr)
                break
            if not raw:
                continue
            self._line_count += 1
            if self._line_count <= 3:
                # Diagnostic: print the first few raw lines so we can
                # confirm the read path is alive and what the chip is
                # actually sending (incl. any boot-time banners).
                print(f"\n[serial:raw #{self._line_count}] {raw!r}", flush=True)
            try:
                line = raw.decode("utf-8", errors="replace")
            except Exception as e:
                print(f"\n[serial] decode error: {e}", file=sys.stderr)
                continue
            parse_inbound_line(line, self._state)


# ---------------------------- Threshold / offline trip logic ----------------------------
def maybe_auto_warn(state: ArduinoState,
                    last_warned: dict,
                    up_min: int, up_max: int,
                    down_min: int, down_max: int) -> str | None:
    """Return a sensor name ("up"/"down") if we should auto-send a WARN, else None.

    Trigger sources, in priority order:
      1. Sensor is offline (TOF_OFFLINE == 0xFFFF).
      2. Sensor reading is outside its in-band window.
      3. Latch reason from the Arduino says "up" or "down" -- already tripped.

    `last_warned[sensor]` is an edge-trigger debouncer so we don't spam
    WARN:sensor on every 30 Hz tick while the condition persists.
    """
    if state.last_received_at == 0.0 or not state.tof_enabled:
        # No telemetry yet, or the operator deliberately disabled TOF.
        # Disabled sensors publish 0xFFFF placeholders, which must not be
        # mistaken for an offline safety fault.
        return None

    for sensor, value, lo, hi in (
        ("up",   state.tof_up,   up_min,   up_max),
        ("down", state.tof_down, down_min, down_max),
    ):
        already = state.latched and state.latch_reason == sensor
        if already:
            last_warned[sensor] = True
            continue

        offline = value == TOF_OFFLINE
        outband = value not in (TOF_OFFLINE,) and (value < lo or value > hi)
        if offline or outband:
            if not last_warned.get(sensor, False):
                last_warned[sensor] = True
                reason = "offline" if offline else f"{value} mm outside [{lo},{hi}]"
                print(f"\n[auto-warn] TOF {sensor} {reason} -> sending WARN:{sensor}")
                return sensor
        else:
            last_warned[sensor] = False
    return None


# ---------------------------- Main loop ----------------------------
def stream_loop(ser: serial.Serial | None,
                pad,
                controller_mode: bool,
                up_min: int, up_max: int, down_min: int, down_max: int) -> None:
    state = ArduinoState()
    reader: SerialReader | None = None
    if ser is not None:
        reader = SerialReader(ser, state)
        reader.start()

    drive_direction = "FORWARD"
    prev_channels = [0] * 21
    active_command = "NONE"
    hold_counter = 0
    last_warned: dict = {"up": False, "down": False}
    _write_count = 0
    last_tof_up = -1
    last_tof_down = -1

    def just_pressed(idx):
        return channels[idx] == 1 and prev_channels[idx] == 0

    clock = pygame.time.Clock()
    print("🚀 Streaming parsed logic to Arduino... Press Ctrl+C to stop.\n")
    try:
        while True:
            if controller_mode:
                # SDL2 controller state in pygame 2.6 is NOT refreshed by
                # event.pump() alone -- we must call pad.update() to pull
                # the latest snapshot from the SDL2 gamepad buffer. Without
                # this, get_axis()/get_button() return stale (zero) values
                # no matter how hard you press.
                pygame.event.pump()
                try:
                    pad.update()
                except Exception:
                    pass
                channels = get_21_channels(pad)
            else:
                # Test mode without controller: zeroed channels. Allows the
                # serial path to keep running so you can exercise the
                # protocol without a gamepad attached.
                channels = [0] * 21

            # --- 1. Direction (L1 / R1) ---
            if channels[15] == 1:
                drive_direction = "FORWARD"
            elif channels[16] == 1:
                drive_direction = "BACKWARD"

            # --- 2. Speed (L2) ---
            drive_speed = channels[4]

            # --- 3. D-Pad overrides ---
            if channels[11] == 1:
                drive_speed = 204
                drive_direction = "FORWARD"
            elif channels[12] == 1:
                drive_speed = 204
                drive_direction = "BACKWARD"

            # --- 4. Altitude control mode (separate field) ---
            # Right trigger (R2) > 10 -> hold_alt mode
            # Otherwise -> manual mode
            altitude_mode = "manual"
            if channels[5] > 10:  # R2 trigger
                altitude_mode = "hold_alt"

            # --- 5. Button command pulses (preserved from prelim script) ---
            new_cmd = None
            if just_pressed(6):       # Cross
                new_cmd = "down_detach"
            elif just_pressed(7):     # Circle
                new_cmd = "down_attach"
            elif just_pressed(8):     # Square
                new_cmd = "up_attach"
            elif just_pressed(9):     # Triangle
                new_cmd = "up_detach"
            elif just_pressed(17):    # Share
                new_cmd = "both_attach"
            elif just_pressed(18):    # Options
                new_cmd = "both_detach"
            elif just_pressed(13):    # D-pad left
                new_cmd = "axon_open"
            elif just_pressed(14):    # D-pad right
                new_cmd = "axon_close"
            elif just_pressed(10):    # PS_BTN -> emergency stop
                new_cmd = "estop"

            if new_cmd is not None:
                active_command = new_cmd
                hold_counter = HOLD_FRAMES

            if hold_counter > 0:
                command_out = active_command
                hold_counter -= 1
            else:
                command_out = "NONE"
                active_command = "NONE"

            prev_channels = list(channels)

            # --- 5. Auto WARN on offline / out-of-band TOF ---
            warn_sensor = maybe_auto_warn(state, last_warned,
                                          up_min, up_max,
                                          down_min, down_max)
            if warn_sensor and ser is not None:
                # The Arduino's parseAndExecuteWarning() treats this as a
                # safety latch. Don't bypass it -- the Arduino is the
                # authority on whether the motors are running.
                ser.write(f"WARN:{warn_sensor}\n".encode("utf-8"))

            # --- 6. Send the drive stream ---
            if command_out not in VALID_COMMANDS:
                # Defensive: should be unreachable, but if a future bind
                # ever introduces a typo we don't want to feed junk to the
                # Arduino and trigger its unknown-command ACK path.
                command_out = "NONE"
            payload = f"{drive_speed},{drive_direction},{command_out},{altitude_mode}\n"
            if ser is not None:
                # Print the literal payload (repr) and flush+reset_input_buffer
                # so pyserial doesn't smear a write across two readStringUntil
                # windows on the ESP side -- which is the classic way the
                # chip sees "0,FORW" then "ARD,NONE,manual\n" and rejects the
                # first fragment as malformed CSV.
                _sent = payload.encode("utf-8")
                if _write_count < 5:
                    print(f"\n[serial:tx #{_write_count + 1}] {_sent!r}", flush=True)
                _write_count += 1
                ser.write(_sent)
                ser.flush()
                # 5 ms gap so macOS's USB CDC driver finishes the current
                # transaction before we queue the next one. Without this
                # the OS can split a 22-byte payload across two USB
                # transactions, which the ESP then reads as two fragments.
                time.sleep(0.005)

            # --- 7. Operator-visible status line ---
            tof_up_disp   = _format_tof(state.tof_up, state.tof_enabled)
            tof_down_disp = _format_tof(state.tof_down, state.tof_enabled)
            latch_disp    = ""
            if state.latched:
                latch_disp = f"  LATCHED({state.latch_reason or '?'})"
            sys.stdout.write(
                f"\r\033[K"
                f"[TOF up={tof_up_disp} down={tof_down_disp}]"
                f" [ack={state.last_ack}]{latch_disp}  "
                f"=> [SERIAL] {payload.strip()}"
            )
            sys.stdout.flush()

            # Also surface a sensor transition in the log so the operator
            # can scroll back and see when each sensor went offline. Cheap
            # because we only do it on change.
            if state.tof_up != last_tof_up:
                last_tof_up = state.tof_up
                print(f"\n[tof.up] -> {tof_up_disp}")
            if state.tof_down != last_tof_down:
                last_tof_down = state.tof_down
                print(f"\n[tof.down] -> {tof_down_disp}")

            clock.tick(SEND_RATE)

    except KeyboardInterrupt:
        print("\n\nStopping stream...")
    finally:
        if reader is not None:
            reader.stop()
        if ser is not None:
            # On shutdown, send a couple of zero-speed frames so the
            # Arduino ends with motors braked -- not in whatever direction
            # the stick was last pushed.
            for _ in range(3):
                ser.write(b"0,FORWARD,NONE\n")
            ser.close()
        pygame.quit()
        print("Cleaned up and shut down.")


# ---------------------------- Entry point ----------------------------
def main():
    ap = argparse.ArgumentParser(description=(
        "Direct gamepad -> Arduino USB-serial bridge. Talks the full "
        "arduino_bridge.ino protocol (CSV drive stream + WARN: + estop) "
        "and surfaces the Arduino's STS/ACK/WARNED replies in the console."
    ))
    ap.add_argument("--test", action="store_true",
                    help="No serial connection. Lets you exercise the "
                         "keybind logic with the controller in isolation.")
    ap.add_argument("--controller", action="store_true",
                    help="Test mode WITH a controller -- serial is skipped "
                         "but the gamepad still drives the printed payload.")
    ap.add_argument("--port", default=None,
                    help="Serial port (default: auto-detect on macOS)")
    ap.add_argument("--baud", type=int, default=BAUD_RATE,
                    help=f"Baud rate (default {BAUD_RATE})")
    ap.add_argument("--tof-up-min",   type=int, default=TOF_UP_MIN_MM)
    ap.add_argument("--tof-up-max",   type=int, default=TOF_UP_MAX_MM)
    ap.add_argument("--tof-down-min", type=int, default=TOF_DOWN_MIN_MM)
    ap.add_argument("--tof-down-max", type=int, default=TOF_DOWN_MAX_MM)
    args = ap.parse_args()

    test_mode = args.test
    ser = None

    if not test_mode:
        print("[main] calling find_arduino_port()...", flush=True)
        port_name = args.port or find_arduino_port()
        print(f"[main] auto-detected port_name = {port_name!r}", flush=True)
        if not port_name:
            print("⚠️  Could not auto-detect Arduino.")
            user_choice = input("Run in Test Mode without Arduino? (y/n): ").strip().lower()
            if user_choice == 'y':
                test_mode = True
            else:
                port_name = input("Enter custom serial port path: ").strip()
        if not test_mode:
            try:
                print(f"[main] calling serial.Serial({port_name!r}, {args.baud}, timeout=2)...", flush=True)
                # The timeout= here only governs read(); on macOS the open()
                # call itself can block if the CDC driver is wedged. We give
                # it a watchdog thread so we don't hang the whole script.
                open_done = threading.Event()
                open_error: Exception | None = None
                def _open():
                    nonlocal ser, open_error
                    try:
                        ser = serial.Serial(
                            port_name, args.baud, timeout=0.5,
                            rtscts=False, dsrdtr=False,
                        )
                        # CRITICAL: prevent pyserial from holding RTS/DTR
                        # high, which on ESP32 dev boards with a DTR->EN
                        # circuit resets the chip into the bootloader. The
                        # bootloader emits no telemetry, so the read path
                        # looks dead even though the firmware is fine.
                        # Modern pyserial moved these out of the
                        # constructor and into post-init attributes.
                        ser.rts = False
                        ser.dtr = False
                    except Exception as e:
                        open_error = e
                    finally:
                        open_done.set()
                t = threading.Thread(target=_open, daemon=True)
                t.start()
                if not open_done.wait(timeout=5.0):
                    print(f"[main] serial.Serial() open() did not return within 5s. The port is wedged.", flush=True)
                    print(f"[main] Likely cause: macOS CDC driver or another process is holding {port_name}.", flush=True)
                    print(f"[main] Try:  lsof | grep {port_name.split('/')[-1]}", flush=True)
                    print(f"[main] Try:  Unplug the Arduino, wait 3s, replug, and re-run.", flush=True)
                    sys.exit(1)
                t.join()
                if open_error is not None:
                    print(f"❌ Serial connection failed: {open_error}")
                    sys.exit(1)
                if ser is None:
                    print("❌ Serial connection failed: serial.Serial() returned None without raising.")
                    sys.exit(1)
                print(f"[main] serial.Serial() returned: {ser}", flush=True)
                time.sleep(2)
                print(f"[main] after 2s settle sleep, ser.is_open={ser.is_open}", flush=True)
                print(f"🔌 Connected to Arduino on {port_name} @ {args.baud} baud")
            except Exception as e:
                print(f"❌ Serial connection failed: {e}")
                sys.exit(1)

    if test_mode:
        print("🧪 RUNNING IN TEST MODE "
              f"({'no controller' if not args.controller else 'controller only, no serial'})\n")

    pad = None
    controller_mode = False
    print("[main] before pygame.init()", flush=True)
    pygame.init()
    print("[main] after pygame.init()", flush=True)
    controller.init()
    print("[main] after controller.init()", flush=True)
    n = controller.get_count()
    print(f"[main] controller.get_count() = {n}", flush=True)
    for i in range(n):
        is_c = controller.is_controller(i)
        print(f"[main] index {i}: is_controller={is_c}", flush=True)
        if is_c:
            print(f"[main] index {i}: creating Controller object...", flush=True)
            pad = controller.Controller(i)
            print(f"[main] index {i}: calling pad.init() (this can hang on macOS)...", flush=True)
            pad.init()
            print(f"[main] index {i}: pad.init() returned", flush=True)
            controller_mode = True
            print(f"🎮 Connected Controller: {pad.name}")
            break
    if not controller_mode and not args.test:
        print("❌ No PS4 controller detected! Ensure it is paired/connected and try again.")
        if ser is not None:
            ser.close()
        sys.exit(1)
    if not controller_mode:
        print("🎮 No controller -- generating zeroed channels every frame.")

    stream_loop(
        ser=ser,
        pad=pad,
        controller_mode=controller_mode,
        up_min=args.tof_up_min,
        up_max=args.tof_up_max,
        down_min=args.tof_down_min,
        down_max=args.tof_down_max,
    )


if __name__ == "__main__":
    main()