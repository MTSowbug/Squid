#!/usr/bin/env python3
"""
Okolab MOT-CONTROLLER utility

Features
- Diagnostics (default)
- Lid control: --open / --close (forces edge if needed)
- Plate control: --release-plate / --lock-plate
- Set control mode: --set-mode {0,1,2}
- Debug polling and adjustable timeouts

IDs used:
 56 R  Communication control mode (0=digital, 1=serial, 2=both)
 57 W  Communication control mode
 33 R  Operation Switch (1=Manual-Open, 2=Manual-Close, 3=Remote)
 71 R  Lid motor serial control
 72 W  Lid motor serial control (0=close, 1=open)
 11 R  Lid closing limit switch  (1=closed)
 12 R  Lid opening  limit switch (1=opened)
 28 R  Lid opening procedure     (1=in progress)
 29 R  Lid closing procedure     (1=in progress)
 74 R  Release Plate serial control
 75 W  Release Plate serial control (0=lock, 1=release)
 13 R  Lid motor direction (0=closing, 1=opening)
"""

from __future__ import annotations
import argparse, sys, time
import serial

CR = b"\r"

class ProtocolError(RuntimeError):
    def __init__(self, code:int) -> None:
        super().__init__(f"Device error E{code}")
        self.code = code

class MOTController:
    def __init__(self, port:str, timeout:float=0.6) -> None:
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
            # E<code>
            try:
                code = int(reply[1:] or b"0")
            except ValueError:
                code = 8
            raise ProtocolError(code)

        if not reply.startswith(payload):
            # reply that doesn't echo request prefix => generic fault
            raise ProtocolError(8)

        return reply[len(payload):]  # value only

    def read(self, cmd_id:int) -> str:
        return self._exchange(cmd_id).decode("ascii")

    def write(self, cmd_id:int, value:int|str) -> None:
        self._exchange(cmd_id, str(value))

    # ----- helpers for control that needs an "edge" -----------------------------
    def _force_edge_then_set(self, read_id:int, write_id:int, target:int, sleep_s:float=0.05) -> None:
        """Ensure a transition is seen: if current == target, flip then set target."""
        cur = self.read(read_id)
        if cur == str(target):
            self.write(write_id, 1 - target)
            time.sleep(sleep_s)
        self.write(write_id, target)

    # ----- high-level operations ------------------------------------------------
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

    def open_lid(self, wait:bool=True, timeout:float=20.0, debug:bool=False) -> None:
        self._force_edge_then_set(71, 72, 1)
        if not wait:
            return
        self._wait_for_limit(limit_id=12, expected="1", timeout=timeout, debug=debug, direction="open")

    def close_lid(self, wait:bool=True, timeout:float=20.0, debug:bool=False) -> None:
        self._force_edge_then_set(71, 72, 0)
        if not wait:
            return
        self._wait_for_limit(limit_id=11, expected="1", timeout=timeout, debug=debug, direction="close")

    def _wait_for_limit(self, limit_id:int, expected:str, timeout:float, debug:bool, direction:str) -> None:
        t0 = time.monotonic()
        last_print = 0.0
        while time.monotonic() - t0 < timeout:
            lim = self.read(limit_id)
            if lim == expected:
                return
            if debug and time.monotonic() - last_print > 0.5:
                # Print a compact status line
                s = {
                    "LID_OPEN_LIM(12)": self.read(12),
                    "LID_CLOSE_LIM(11)": self.read(11),
                    "DIR(13)": self.read(13),
                    "OPENING_PROC(28)": self.read(28),
                    "CLOSING_PROC(29)": self.read(29),
                    "SERCTL(71)": self.read(71),
                }
                print("status:", " ".join(f"{k}={v}" for k, v in s.items()))
                last_print = time.monotonic()
            time.sleep(0.2)
        raise TimeoutError(f"lid movement timeout ({direction})")

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
    ap = argparse.ArgumentParser(description="Okolab MOT-CONTROLLER tool")
    ap.add_argument("port", help="e.g. /dev/ttyACM0")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--open",  action="store_true", help="open lid")
    g.add_argument("--close", action="store_true", help="close lid")
    g.add_argument("--release-plate", action="store_true", help="release plate")
    g.add_argument("--lock-plate",    action="store_true", help="lock plate")
    ap.add_argument("--set-mode", type=int, choices=[0,1,2], help="set communication control mode")
    ap.add_argument("--no-wait", action="store_true", help="do not wait for completion")
    ap.add_argument("--timeout", type=float, default=20.0, help="movement timeout seconds")
    ap.add_argument("--debug", action="store_true", help="print status while waiting")
    args = ap.parse_args(argv)

    try:
        ctrl = MOTController(args.port)
    except serial.SerialException as exc:
        sys.exit(f"Cannot open serial port: {exc}")

    try:
        # Preflight hints if we are about to control the lid
        do_control = args.open or args.close or args.release_plate or args.lock_plate or args.set_mode is not None

        if args.set_mode is not None:
            ctrl.set_comm_mode(args.set_mode)

        if args.release_plate:
            ctrl.release_plate(wait=not args.no_wait)
        elif args.lock_plate:
            ctrl.lock_plate(wait=not args.no_wait)
        elif args.open:
            # sanity checks
            if ctrl.read(33) != "3":
                sys.exit("Operation Switch not in Remote (ID 33 != 3). Set the switch to Remote.")
            mode = ctrl.read(56)
            if mode not in ("1","2"):
                sys.exit(f"Communication control mode is {mode}; set to 1 or 2 with --set-mode.")
            ctrl.open_lid(wait=not args.no_wait, timeout=args.timeout, debug=args.debug)
        elif args.close:
            if ctrl.read(33) != "3":
                sys.exit("Operation Switch not in Remote (ID 33 != 3). Set the switch to Remote.")
            mode = ctrl.read(56)
            if mode not in ("1","2"):
                sys.exit(f"Communication control mode is {mode}; set to 1 or 2 with --set-mode.")
            ctrl.close_lid(wait=not args.no_wait, timeout=args.timeout, debug=args.debug)
        else:
            # Diagnostics
            diag = ctrl.diagnostics()
            w = max(map(len, diag))
            for k, v in diag.items():
                print(f"{k:<{w}} : {v}")
    except (TimeoutError, ProtocolError) as exc:
        sys.exit(str(exc))
    finally:
        ctrl.close()

if __name__ == "__main__":
    main()
