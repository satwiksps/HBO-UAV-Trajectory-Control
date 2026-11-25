"""Illustrative one-axis plant. Its traces are never hardware measurements."""
import math
from dataclasses import replace

import numpy as np

from rig_link import Calibration, DEFAULT_CAP, GAINS, HARD_CAP, IDLE_US, LIFT_US, Sample

SIM_CAL = Calibration((0, 0, 1), (-1, 0, 0), (0, 0, 0), -30, 40, LIFT_US, 1138)
LAUNCH_KP, LAUNCH_KD = 0.55, 0.25
APPROACH_BRAKE_GAIN = 0.70
LAUNCH_CAPTURE_ERROR, LAUNCH_CAPTURE_RATE = 3.0, 5.0


class SimRig:
    def __init__(self, seed=71):
        self.calibration = SIM_CAL
        self.control_upper = SIM_CAL.upper
        self.calibrated = True
        self.random = np.random.default_rng(seed)
        self.angle = SIM_CAL.lower
        self.rate = 0.0
        self.motor = 0.0
        self.integral = 0.0
        self.launch_reference = None
        self.launch_boost = 0.0
        self.terms = (0.0, 0.0, 0.0)
        self.dip_start = None
        self.dip = 0
        self.target = 0.0
        self.gains = GAINS
        self.pulse = self.manual = IDLE_US
        self.mode = self.fault = 0
        self.armed = False
        self.t = self.last_frame = self.started = self.heartbeat = self.last_slew = 0.0
        self.seq = 0
        self.acks = []
        self.extra_load = 0.0
        self.top_since = None
        self.held_angle = None

    def send(self, command):
        key, *values = command.split()
        values = list(map(float, values))
        ok = True
        if key == "H":
            self.heartbeat = self.t
            return
        if key in ("X", "S"):
            self.stop()
            if key == "X":
                self.armed = False
        elif key == "A" and not self.armed and self.angle < self.control_upper-4:
            self.armed, self.fault = True, 0
        elif key == "K" and not self.armed:
            self.calibration = replace(self.calibration, zero_axis=tuple(values[:3]),
                                       tangent=tuple(values[3:6]), bias=tuple(values[6:9]),
                                       lower=values[9])
            self.control_upper = values[10]
            self.calibrated, self.target = True, 0.0
        elif key == "L" and not self.armed and 1120 <= values[0] < values[1] <= HARD_CAP:
            self.calibration = replace(self.calibration, hover=int(values[0]), cap=int(values[1]))
        elif key == "G" and self.mode == 0:
            self.gains = tuple(values)
        elif (key == "J" and self.armed and self.mode == 1 and self.launch_reference is None
              and abs(self.target-self.angle) < 3 and abs(self.rate) < 12):
            e = self.target-self.angle
            kp, _, kd = self.gains
            previous = kp*e + self.integral - kd*self.rate
            self.gains = tuple(values)
            kp, ki, kd = self.gains
            self.integral = min(8, max(-8, previous-kp*e+kd*self.rate)) if ki > 0 else 0
        elif (key == "D" and self.armed and self.mode == 1 and self.launch_reference is None
              and self.angle < self.control_upper-6 and abs(self.target-self.angle) < 5
              and abs(self.rate) < 8 and (self.dip_start is None or self.t-self.dip_start >= 15)):
            self.dip_start = self.t
        elif key == "T" and self.calibration.targets[0] <= values[0] <= self.control_upper-4:
            self.target = values[0]
        elif key == "R" and self.armed and self.angle < self.control_upper-3:
            if self.mode != 1:
                self.started, self.integral, self.mode = self.t, 0.0, 1
                self.launch_reference = self.angle
                self.launch_boost = 0.0
        elif key == "P" and self.armed and 1100 <= values[0] <= self.calibration.cap and self.angle < self.control_upper-2:
            if self.mode != 2:
                self.started = self.t
                self.launch_reference = None
                self.dip_start = None
                self.dip = 0
                self.terms = (0.0, 0.0, 0.0)
            self.manual, self.mode = int(values[0]), 2
        else:
            ok = False
        self.acks.append((key, ok))

    def stop(self):
        self.mode, self.pulse, self.integral, self.top_since = 0, IDLE_US, 0, None
        self.launch_reference = None
        self.launch_boost = 0.0
        self.terms = (0.0, 0.0, 0.0)
        self.dip_start = None
        self.dip = 0

    def trip(self, code):
        self.stop()
        self.fault, self.armed = code, False

    def poke(self):
        self.rate -= 28

    def weight(self):
        self.extra_load = 8.0 if self.extra_load == 0 else 0.0

    def step(self, dt):
        self.t += dt
        c = self.calibration
        if self.armed and self.t - self.heartbeat > 0.75:
            self.trip(1)
        if self.mode and self.t-self.started > 180:
            self.trip(6)
        if self.mode and abs(self.rate) > 160:
            self.trip(7)
        if self.mode and self.angle > self.control_upper+0.5:
            self.trip(4)
        if (self.mode and self.rate > 0 and self.angle >= self.control_upper-4
                and self.angle + 0.14*self.rate >= self.control_upper+0.5):
            self.trip(4)
        if self.mode and self.angle >= self.control_upper-1.5:
            self.top_since = self.t if self.top_since is None else self.top_since
            if self.t-self.top_since >= 0.12:
                self.trip(4)
        else:
            self.top_since = None
        desired = self.manual if self.mode == 2 else IDLE_US
        if self.mode == 1:
            self.dip = -4 if self.dip_start is not None and self.t-self.dip_start < 2 else 0
            kp, ki, kd = self.gains
            e = self.target - self.angle
            approach_rate = 2.0 + 0.65*max(e, 0)
            approach_brake = -APPROACH_BRAKE_GAIN*max(self.rate-approach_rate, 0)
            if self.launch_reference is not None:
                goal = min(self.angle + 8, max(self.angle - 8, self.target))
                self.launch_reference += min(12*dt, max(-12*dt, goal - self.launch_reference))
                launch_error = min(8, max(-8, self.launch_reference-self.angle))
                if (abs(self.launch_reference-self.target) < 0.1 and abs(e) <= LAUNCH_CAPTURE_ERROR
                        and abs(self.rate) <= LAUNCH_CAPTURE_RATE):
                    if ki > 0:
                        transfer = self.launch_boost + LAUNCH_KP*launch_error - kp*e + (kd-LAUNCH_KD)*self.rate
                        self.integral = min(8, max(-8, transfer))
                    self.launch_reference = None
                elif self.t-self.started >= 15:
                    self.trip(8)
                else:
                    if (e > LAUNCH_CAPTURE_ERROR and launch_error >= LAUNCH_CAPTURE_ERROR
                            and abs(self.rate) <= 4 and self.pulse >= c.hover):
                        self.launch_boost = min(8, self.launch_boost+1.2*dt)
                    elif self.rate > LAUNCH_CAPTURE_RATE or launch_error <= 0:
                        self.launch_boost = max(0, self.launch_boost-10*dt)
                    kp, kd, e = LAUNCH_KP, LAUNCH_KD, launch_error
            if self.mode == 1:
                if self.launch_reference is None:
                    trial_i = min(8, max(-8, self.integral + ki*e*dt))
                    estimate = c.hover + kp*e + trial_i - kd*self.rate + self.dip + approach_brake
                    lagging = self.pulse >= 1100 and estimate > self.pulse+1
                    if (((not lagging and estimate <= c.cap) or trial_i < self.integral)
                            and (estimate >= 1100 or trial_i > self.integral)):
                        self.integral = trial_i
                correction = self.integral if self.launch_reference is None else self.launch_boost
                derivative = -kd*self.rate
                self.terms = (kp*e, self.integral, derivative)
                command = c.hover+kp*e+correction+derivative+self.dip+approach_brake
                desired = round(min(c.cap, max(1100, command)))
        if desired <= self.pulse:
            self.pulse = desired
        elif self.t - self.last_slew >= 0.019:
            self.last_slew = self.t
            self.pulse = min(desired, max(1100, self.pulse+1))
        self.motor += (max(0, self.pulse-1090)-self.motor) * dt/0.115
        if self.held_angle is not None and self.mode == 0:
            self.angle, self.rate = self.held_angle, 0.0
            return
        # Local illustrative torque model, independent of hardware measurements.
        force = 6.6*(self.motor-40) - 0.28*self.angle - self.extra_load - 2.6*self.rate
        force -= 2.0*math.tanh(self.rate/0.7)
        force += self.random.normal(0, 6.0)
        self.rate += force*dt
        self.angle += self.rate*dt
        if self.angle <= c.lower:
            self.angle, self.rate = c.lower, max(0, self.rate)
        elif self.angle >= c.upper:
            self.angle, self.rate = c.upper, min(0, self.rate)

    def poll(self, dt):
        rows = []
        steps = max(1, round(min(dt, 0.20)/0.005))
        for _ in range(steps):
            self.step(dt/steps)
            if self.t-self.last_frame < 0.0199:
                continue
            self.last_frame = self.t
            self.seq += 1
            a = math.radians(self.angle)
            raw = (-16384*math.sin(a), 0, 16384*math.cos(a), 0, 131*self.rate, 0)
            raw = tuple(round(v) for v in raw)
            flags = (1 | 16 | (6 if self.calibrated else 0) | (8 if self.armed else 0)
                     | (32 if self.mode == 1 and self.launch_reference is not None else 0))
            rows.append(Sample(round(1000*self.t), self.seq, flags, self.mode, self.fault,
                               self.pulse, self.calibration.cap, self.target,
                               self.angle+self.random.normal(0, 0.06),
                               self.angle+self.random.normal(0, 0.35), self.rate, raw, 0x68, self.terms, self.dip))
        return rows

    def close(self):
        self.send("X")
