#!/usr/bin/env python3
"""
Okolab MOT-CONTROLLER tool

Additions in this version
- Auto-detects the serial port; no positional PORT argument required.
- Detection order:
  1) --port ARG (if provided) or $OKOLAB_PORT
  2) /dev/serial/by-id/* symlinks matching vendor/product text (stable path)
  3) VID:PID match via pyserial.tools.list_ports (03eb:2404 from dmesg you shared)
  4) First available /dev/ttyACM* as a last resort

You can still pass --port to override.

Existing features
- Diagnostics (default)
- Lid: --open / --close (edge-forced)
- Plate: --release-plate / --lock-plate
- --set-mode {0,1,2}, --timeout, --no-wait, --debug
"""

from __future__ import annotations
import argparse, os, re, sys, time, glob, pathlib
import serial
from serial.tools import list_ports

CR = b"\r"

# Known identifiers for robustness
KNOWN_BYID_PATTERNS = [
    r"ATMEL.*CDC[_ ]Virtual[_ ]Com",  # matches your by-id name
    r"Okolab",                        # future-proofing if vendor string changes
]
KNOWN_VID = 0x03EB  # Atmel/Microchip (from your dmesg: idVendor=03eb)
KNOWN_PID = 0x2404  # from your dmesg: idProduct=2404

class ProtocolError(RuntimeError):
    def __init__(self, code:int) -> None:
        super().__init__(f"Device error E{code}")
        self.code = code

def iter_by_id_candidates() -> list[str]:
    paths = sorted(glob.glob("/dev/serial/by-id/*"))
    if not paths:
        return []
    # Prefer matching names; keep all as fallback
    pri = []
    sec = []
    for p in paths:
        name = os.path.basename(p)
        if any(re.search(pat, name, flags=re.IGNORECASE) for pat in KNOWN_BYID_PATTERNS):
            pri.append(p)
        else:
            sec.append(p)
    return pri + sec

def find_port(explicit: str | None) -> str:
    # 0) explicit arg or environment
    env = os.environ.get("OKOLAB_PORT")
    if explicit:
        return explicit
    if env:
        return env

    # 1) stable udev by-id symlinks (prefer)
    for p in iter_by_id_candidates():
        try:
            # Use the symlink itself; pyserial can open it and it stays stable
            if os.path.exists(p):
                return p
        except OSError:
            pass

    # 2) VID:PID via pyserial enumeration
    for info in list_ports.comports():
        if (info.vid, info.pid) == (KNOWN_VID, KNOWN_PID):
            return info.device  # e.g. /dev/ttyACM1

    # 3) fallback: first ttyACM*
    acms = sorted(glob.glob("/dev/ttyACM*"))
    if acms:
        return acms[0]

    raise SystemExit("No serial port found. Connect the controller or pass --port.")

LID_SERIAL_CTRL = 72
LID_LIMIT_OPEN  = 12
LID_LIMIT_CLOSE = 11

class MOTController:
    def __init__(self, port:str, timeout:float=0.6) -> None:
        self.port = port
        self.ser = serial.Serial(
            port=port, baudrate=115_200,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout, write_timeout=timeout
        )

    @staticmethod
    def _fmt_id(cmd_id:int) -> bytes:
        return f"{cmd_id:03d}".encode("ascii")

    def _tx(self, payload:bytes) -> None:
        self.ser.reset_input_buffer()
        self.ser.write(payload + CR)

    def _rx(self) -> bytes:
        reply = self.ser.read_until(CR)
        if not reply.endswith(CR):
            raise TimeoutError("incomplete reply (missing CR)")
        return reply[:-1]

    def _exchange(self, cmd_id:int, value:str|None=None) -> bytes:
        payload = self._fmt_id(cmd_id) + (value.encode() if value else b"")
        self._tx(payload)
        reply = self._rx()

        if reply.startswith(b"E"):
            try:
                code = int(reply[1:] or b"0")
            except ValueError:
                code = 8
            raise ProtocolError(code)

        if not reply.startswith(payload):
            raise ProtocolError(8)
        return reply[len(payload):]

    def read(self, cmd_id:int) -> str:
        return self._exchange(cmd_id).decode("ascii")

    def write(self, cmd_id:int, value:int|str) -> None:
        self._exchange(cmd_id, str(value))

    def _force_edge_then_set(self, read_id:int, write_id:int, target:int, sleep_s:float=0.05) -> None:
        cur = self.read(read_id)
        if cur == str(target):
            self.write(write_id, 1 - target)
            time.sleep(sleep_s)
        self.write(write_id, target)

    def set_comm_mode(self, mode:int) -> None:
        assert mode in (0,1,2)
        self.write(57, mode)

    def release_plate(self, wait:bool=True, timeout:float=5.0) -> None:
        self._force_edge_then_set(74, 75, 1)
        if wait:
            t0 = time.monotonic()
            while time.monotonic() - t0 < timeout:
                if self.read(74) == "1":
                    return
                time.sleep(0.1)
            raise TimeoutError("plate release timeout")

    def lock_plate(self, wait:bool=True, timeout:float=5.0) -> None:
        self._force_edge_then_set(74, 75, 0)
        if wait:
            t0 = time.monotonic()
            while time.monotonic() - t0 < timeout:
                if self.read(74) == "0":
                    return
                time.sleep(0.1)
            raise TimeoutError("plate lock timeout")

    def _wait_for_limit(self, limit_id:int, expected:str, timeout:float, debug:bool, direction:str) -> None:
        t0 = time.monotonic()
        last_print = 0.0
        while time.monotonic() - t0 < timeout:
            lim = self.read(limit_id)
            if lim == expected:
                return
            if debug and time.monotonic() - last_print > 0.5:
                s = {
                    "LIM_OPEN(12)": self.read(12),
                    "LIM_CLOSE(11)": self.read(11),
                    "DIR(13)": self.read(13),
                    "OPENING(28)": self.read(28),
                    "CLOSING(29)": self.read(29),
                    "SERCTL(71)": self.read(71),
                }
                print("status:", " ".join(f"{k}={v}" for k, v in s.items()))
                last_print = time.monotonic()
            time.sleep(0.2)
        raise TimeoutError(f"lid movement timeout ({direction})")

    def open_lid(self, wait:bool=True, timeout:float=20.0, debug:bool=False) -> None:
        self._force_edge_then_set(71, LID_SERIAL_CTRL, 1)
        if wait:
            self._wait_for_limit(LID_LIMIT_OPEN, "1", timeout, debug, "open")

    def close_lid(self, wait:bool=True, timeout:float=20.0, debug:bool=False) -> None:
        self._force_edge_then_set(71, LID_SERIAL_CTRL, 0)
        if wait:
            self._wait_for_limit(LID_LIMIT_CLOSE, "1", timeout, debug, "close")

    def diagnostics(self) -> dict[str,str]:
        ids = {
            1:"product_code", 2:"serial_number", 3:"production_date",
            5:"software_version", 6:"ist_rev_info", 7:"system_uptime",
            56:"comm_control_mode", 33:"operation_switch_status",
            11:"lid_closed", 12:"lid_opened",
            28:"lid_opening_proc", 29:"lid_closing_proc",
            32:"lid_motor_duty_percent", 35:"plate_locker_duty_percent",
            54:"plate_locker_freq_hz", 14:"multiwell_present",
            71:"lid_serial_control", 74:"plate_serial_control"
        }
        return {name: self.read(cid) for cid, name in ids.items()}

    def close(self) -> None:
        self.ser.close()


def main(argv:list[str]|None=None) -> None:
    ap = argparse.ArgumentParser(description="Okolab MOT-CONTROLLER tool (auto-detects serial port)")
    ap.add_argument("--port", help="override auto-detection (e.g. /dev/ttyACM1 or /dev/serial/by-id/...)")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--open",  action="store_true", help="open lid")
    g.add_argument("--close", action="store_true", help="close lid")
    g.add_argument("--release-plate", action="store_true", help="release plate")
    g.add_argument("--lock-plate",    action="store_true", help="lock plate")
    ap.add_argument("--set-mode", type=int, choices=[0,1,2], help="set communication control mode")
    ap.add_argument("--no-wait", action="store_true", help="do not wait for completion")
    ap.add_argument("--timeout", type=float, default=20.0, help="movement timeout seconds")
    ap.add_argument("--debug", action="store_true", help="print status while waiting")
    ap.add_argument("--list-ports", action="store_true", help="list detected serial ports and exit")
    args = ap.parse_args(argv)

    # Optional listing
    if args.list_ports:
        print("by-id candidates:")
        for p in iter_by_id_candidates():
            target = pathlib.Path(p)
            try:
                print("  ", p, "->", target.resolve())
            except OSError:
                print("  ", p, "(broken symlink)")
        print("pyserial enumeration:")
        for info in list_ports.comports():
            print(f"  {info.device}  vid:pid={info.vid:04x}:{info.pid:04x}  desc={info.description}")
        return

    port = find_port(args.port)
    try:
        ctrl = MOTController(port)
    except serial.SerialException as exc:
        sys.exit(f"Cannot open serial port {port}: {exc}")

    try:
        if args.set_mode is not None:
            ctrl.set_comm_mode(args.set_mode)

        if args.release_plate:
            ctrl.release_plate(wait=not args.no_wait)
        elif args.lock_plate:
            ctrl.lock_plate(wait=not args.no_wait)
        elif args.open:
            if ctrl.read(33) != "3":
                sys.exit("Operation Switch not in Remote (ID 33 != 3).")
            mode = ctrl.read(56)
            if mode not in ("1","2"):
                sys.exit(f"Communication control mode is {mode}; set to 1 or 2 with --set-mode.")
            ctrl.open_lid(wait=not args.no_wait, timeout=args.timeout, debug=args.debug)
        elif args.close:
            if ctrl.read(33) != "3":
                sys.exit("Operation Switch not in Remote (ID 33 != 3).")
            mode = ctrl.read(56)
            if mode not in ("1","2"):
                sys.exit(f"Communication control mode is {mode}; set to 1 or 2 with --set-mode.")
            ctrl.close_lid(wait=not args.no_wait, timeout=args.timeout, debug=args.debug)
        else:
            diag = ctrl.diagnostics()
            w = max(map(len, diag))
            print(f"port : {port}")
            for k, v in diag.items():
                print(f"{k:<{w}} : {v}")
    except (TimeoutError, ProtocolError) as exc:
        sys.exit(str(exc))
    finally:
        ctrl.close()

if __name__ == "__main__":
    main()
