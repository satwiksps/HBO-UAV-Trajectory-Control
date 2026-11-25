"""Board protocol, IMU calibration and signed arm-angle geometry."""
from dataclasses import dataclass
import json
from pathlib import Path
import time

import numpy as np
import serial
from serial.tools import list_ports

IDLE_US = 950
LIFT_US = 1128
START_TRIM_US = 1149
HARD_CAP = 1300
DEFAULT_CAP = HARD_CAP
RIG_MAX_ANGLE = 15.2
GAINS = (0.55, 0.13, 0.25)
FAULTS = ("", "Heartbeat lost", "MPU6050 unavailable", "Tilt estimate unreliable",
          "Upper stop guard", "Angle outside calibrated travel", "Three-minute cutoff",
          "Arm moving too fast", "PID launch did not reach the target")


@dataclass(frozen=True)
class Sample:
    ms: int
    seq: int
    flags: int
    mode: int
    fault: int
    pulse: int
    cap: int
    target: float
    angle: float
    accel_angle: float
    rate: float
    raw: tuple[int, ...]
    addr: int
    terms: tuple[float, float, float] | None = None
    dip: int | None = None

    @property
    def imu(self):
        return bool(self.flags & 1)

    @property
    def angle_valid(self):
        return self.flags & 7 == 7

    @property
    def armed(self):
        return bool(self.flags & 8)

    @property
    def gravity(self):
        return bool(self.flags & 16)

    @property
    def launching(self):
        return bool(self.flags & 32)

    @property
    def error(self):
        return self.target - self.angle


def parse_sample(line):
    fields = line.split(",")
    if (fields[0], len(fields)) not in (("ARM2", 19), ("ARM3", 23)):
        return None
    try:
        a = list(map(int, fields[1:]))
    except ValueError:
        return None
    if (a[0] < 0 or a[1] < 0 or not 0 <= a[2] <= 63 or a[3] not in (0, 1, 2)
            or not 0 <= a[4] < len(FAULTS) or not IDLE_US <= a[5] <= a[6] <= HARD_CAP
            or a[17] not in (0, 0x68, 0x69)):
        return None
    terms = tuple(v/100 for v in a[18:21]) if len(a) == 22 else None
    dip = a[21] if terms is not None else None
    if dip not in (None, 0, -4):
        return None
    return Sample(*a[:7], *(v / 100 for v in a[7:11]), tuple(a[11:17]), a[17], terms, dip)


class SerialRig:
    def __init__(self, port):
        self.port = serial.Serial(baudrate=115200, timeout=0, write_timeout=0.13)
        self.port.port = port
        self.port.rts = self.port.dtr = False
        self.port.open()
        self.buffer = b""
        self.acks = []
        self.started = time.monotonic()

    def send(self, command):
        self.port.write((command + "\n").encode("ascii"))

    def poll(self, _dt):
        self.buffer += self.port.read(min(self.port.in_waiting, 8192))
        *lines, self.buffer = self.buffer.split(b"\n")
        if len(self.buffer) > 1024:
            self.buffer = b""
        samples = []
        for raw in lines:
            line = raw.decode("ascii", errors="replace").strip()
            if line.startswith("ACK,"):
                parts = line.split(",")
                if len(parts) == 3:
                    self.acks.append((parts[1], parts[2] == "1"))
            else:
                sample = parse_sample(line)
                if sample:
                    samples.append(sample)
        return samples

    def close(self):
        try:
            self.send("X")
        finally:
            self.port.close()

    @staticmethod
    def ports():
        return [entry.device for entry in list_ports.comports()]


@dataclass(frozen=True)
class Calibration:
    zero_axis: tuple[float, float, float]
    tangent: tuple[float, float, float]
    bias: tuple[float, float, float]
    lower: float
    upper: float
    hover: int = START_TRIM_US
    cap: int = DEFAULT_CAP
    operating_limit: float | None = None

    @property
    def firmware_upper(self):
        if self.operating_limit is None:
            return self.upper
        # The compiled Uno trips immediately at its stored guard + 0.5°.
        return round(self.operating_limit - 0.5, 2)

    @property
    def targets(self):
        return round(self.lower + 3, 2), round(self.firmware_upper - 4, 2)

    def commands(self):
        values = (*self.zero_axis, *self.tangent, *self.bias, self.lower, self.firmware_upper)
        return ["K " + " ".join(f"{v:.5f}" for v in values), f"L {self.hover} {self.cap}"]

    def save(self, path):
        path = Path(path)
        tmp = path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.__dict__, indent=2) + "\n", encoding="utf-8")
        tmp.replace(path)

    @classmethod
    def load(cls, path):
        return cls(**json.loads(Path(path).read_text(encoding="utf-8")))


def capture_pose(samples):
    if len(samples) < 20 or samples[-1].ms - samples[0].ms < 350:
        raise ValueError("Wait for at least half a second of live IMU samples.")
    if not all(row.imu and row.gravity for row in samples):
        raise ValueError("The MPU6050 is unstable or unavailable.")
    raw = np.array([row.raw for row in samples], dtype=float)
    if np.max(np.std(raw[:, :3], axis=0)) > 500 or np.max(np.ptp(raw[:, 3:], axis=0)) > 360:
        raise ValueError("The arm moved during capture. Hold it still and retry.")
    average = np.mean(raw, axis=0)
    gravity = average[:3] / np.linalg.norm(average[:3])
    return gravity, average[3:]


def calibrate(poses, lower, upper, hover=START_TRIM_US, cap=DEFAULT_CAP, operating_limit=None):
    if not (-85 <= lower <= -6 and 8 <= upper <= 85):
        raise ValueError("Enter measured lower (−85…−6°) and upper (+8…+85°) stops.")
    if not 1120 <= hover < cap <= HARD_CAP or cap <= LIFT_US:
        raise ValueError("Trim must be below the working limit, and the limit must exceed 1128 µs.")
    if operating_limit is not None and not 8.5 <= operating_limit <= upper:
        raise ValueError("The operating ceiling must be between +8.5° and the physical upper stop.")
    a_low, a_zero, a_high = (poses[name][0] for name in ("Lower", "Level", "Upper"))
    n = np.cross(a_low-a_zero, a_high-a_zero)
    if np.linalg.norm(n) < 0.006:
        raise ValueError("The three poses do not form a clear rotation. Capture them again.")
    n /= np.linalg.norm(n)
    x = a_zero - np.dot(a_zero, n)*n
    if np.linalg.norm(x) < 0.65:
        raise ValueError("The axis must tilt relative to gravity for this IMU calibration.")
    x /= np.linalg.norm(x)
    y = np.cross(n, x)
    if np.dot(a_high, y) < 0:
        y = -y
    pred = [np.degrees(np.arctan2(np.dot(a, y), np.dot(a, x))) for a in (a_low, a_high)]
    if abs(pred[0]-lower) > 4 or abs(pred[1]-upper) > 4:
        raise ValueError(f"IMU travel is {pred[0]:.1f}° to {pred[1]:.1f}°. "
                         "Check the stop angles and recapture the three poses.")
    bias = tuple(float(v) for v in poses["Level"][1])
    return Calibration(tuple(map(float, x)), tuple(map(float, y)), bias, lower, upper, hover, cap,
                       operating_limit)
