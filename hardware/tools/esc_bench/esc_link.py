"""Check the Uno output or run one bounded, propeller-off ESC bench test."""
import argparse
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / ".tools" / "python-libs"))
import serial


def send(port, command):
    port.write(f"{command}\n".encode("ascii"))


def receive(port):
    return port.readline().decode("ascii", errors="replace").strip()


def check_idle(port, pin, idle_us):
    send(port, "CHECK")
    fields = receive(port).split(",")
    if len(fields) != 4 or fields[:3] != ["ESC_BENCH_V2", f"D{pin}", str(idle_us)]:
        raise RuntimeError(f"Expected the bench firmware at minimum throttle: {fields}")
    measured_us = int(fields[3])
    if abs(measured_us - idle_us) > 50:
        raise RuntimeError(f"D{pin} minimum pulse readback out of range: {measured_us} µs")
    print(f"D{pin} minimum throttle pulse readback: {measured_us} us", flush=True)


def spin_once(port, pin, idle_us):
    check_idle(port, pin, idle_us)
    print("Starting single 2.8-second throttle test.", flush=True)
    expected = [f"THROTTLE,{value}" for value in (1100, 1150, 1200, 1250, 1300)]
    idle_response = f"IDLE,{idle_us}"
    expected.append(idle_response)
    send(port, "SPIN")
    try:
        deadline = time.monotonic() + 4
        for response in expected:
            while time.monotonic() < deadline:
                reply = receive(port)
                if reply:
                    break
            else:
                raise RuntimeError(f"Timed out waiting for {response}")
            print(reply, flush=True)
            if reply != response:
                raise RuntimeError(f"Expected {response}, received {reply}")
    finally:
        send(port, "STOP")
        port.flush()
    if receive(port) != idle_response:
        raise RuntimeError("The Arduino did not acknowledge STOP.")
    check_idle(port, pin, idle_us)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM5")
    parser.add_argument("--pin", type=int, choices=(6, 9), default=9)
    parser.add_argument("--idle-us", type=int, choices=(950, 1000), default=1000)
    parser.add_argument("--spin", action="store_true",
                        help="Only with the propeller removed and the motor mounted.")
    args = parser.parse_args()
    with serial.Serial(port=None, baudrate=115200, timeout=0.25, write_timeout=1) as port:
        port.port = args.port
        port.dtr = False
        port.rts = False
        port.open()
        # Account for drivers that briefly assert DTR while opening the port.
        time.sleep(5.8 if args.spin else 2)
        port.reset_input_buffer()
        if args.spin:
            spin_once(port, args.pin, args.idle_us)
        else:
            check_idle(port, args.pin, args.idle_us)


if __name__ == "__main__":
    main()
