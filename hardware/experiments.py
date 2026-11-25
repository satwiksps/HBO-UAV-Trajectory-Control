"""Experiment sequencing. A single supervisor owns all actuator commands."""
from concurrent.futures import ThreadPoolExecutor
import csv
from dataclasses import asdict, dataclass
from datetime import datetime
import json
from pathlib import Path

import numpy as np

from hbo import BOUNDS, Models, Score, gains_at, score_trial, unit
from rig_link import FAULTS, GAINS, HARD_CAP, IDLE_US

ROOT = Path(__file__).resolve().parent
TRIAL_PHASES = (("Rest", 3), ("Level", 4), ("Up", 6), ("Down", 6))
MATCHED_HBO = "Matched HBO"
SCORED_EXPERIMENTS = ("Baseline", "HBO", "Repeat", MATCHED_HBO)
MATCHED_PAIRS = 5
LIVE_TARGET_INTERVAL = 0.2
LIVE_TARGET_STEP = 0.4
HOLD_OVERSHOOT_DEG = 6.0
HOLD_OVERSHOOT_MS = 200
DEMO_START, DEMO_NEXT = -9.0, -6.0
LOG_FIELDS = (
    "source", "elapsed_s", "device_ms", "sample", "experiment", "trial", "group", "phase",
    "phase_s", "angle_deg", "target_deg", "error_deg", "accel_angle_deg", "gyro_deg_s",
    "esc_us", "limit_us", "mode", "armed", "imu_ok", "angle_valid", "gravity_ok",
    "fault", "kp", "ki", "kd", "ax", "ay", "az", "gx", "gy", "gz", "launching",
    "p_us", "i_us", "d_us", "disturbance_us",
)


@dataclass(frozen=True)
class GuidedComparison:
    source_run: str
    trim: int
    targets: tuple[float, float, float]
    baseline: tuple[float, float, float]
    candidate: tuple[float, float, float]

    @classmethod
    def load(cls, path):
        path = Path(path)
        metadata = json.loads(path.read_text(encoding="utf-8"))
        prior = metadata["live_guided_check"]
        if metadata["source"] != "HARDWARE" or prior["test_trim_us"] != metadata["calibration"]["hover"]:
            raise ValueError("Choose a hardware guided-candidate session with consistent trim metadata.")
        targets = tuple(float(value) for value in metadata["trial_targets_deg"])
        trim = int(prior["test_trim_us"])
        candidate = tuple(round(float(value), 5) for value in prior["candidate"])
        with path.with_name("trials.csv").open(newline="", encoding="utf-8") as file:
            rows = list(csv.DictReader(file))
        if any(row["source"] != "HARDWARE" for row in rows):
            raise ValueError("The trial log contains inconsistent data sources.")
        baseline = {tuple(float(row[key]) for key in ("kp", "ki", "kd")) for row in rows
                    if row["experiment"] == "Baseline" and row["fault"] == "0"}
        guided = {tuple(float(row[key]) for key in ("kp", "ki", "kd")) for row in rows
                  if row["experiment"] == "HBO" and row["group"] == "Guided" and row["fault"] == "0"}
        if len(targets) != 3 or not np.all(np.isfinite(targets)) or not 1120 <= trim < HARD_CAP:
            raise ValueError("The guided session has an invalid trim or target profile.")
        if len(baseline) != 1 or candidate not in guided or candidate in baseline:
            raise ValueError("The log must contain one manual gain set and a completed guided candidate.")
        manual = baseline.pop()
        for gains in (manual, candidate):
            if len(gains) != 3 or not np.all((BOUNDS[:, 0] <= gains) & (gains <= BOUNDS[:, 1])):
                raise ValueError("Both saved gain sets must be inside the existing HBO search bounds.")
        return cls(path.parent.name, trim, targets, manual, candidate)


class Session:
    def __init__(self, simulated, base=None):
        self.simulated = simulated
        self.source = "SIMULATION" if simulated else "HARDWARE"
        stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S_%f")
        self.path = (base or ROOT / "runs") / self.source.lower() / stamp
        self.path.mkdir(parents=True)
        self.stream = (self.path / "telemetry.csv").open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.stream)
        self.writer.writerow(LOG_FIELDS)
        self.trials = []
        self.events = []
        self.metadata = {"source": self.source, "recorded_at": datetime.now().astimezone().isoformat()}
        self.write_metadata()

    def write_metadata(self):
        (self.path / "session.json").write_text(json.dumps(self.metadata, indent=2), encoding="utf-8")

    def log(self, elapsed, sample, experiment, trial, group, phase, phase_s, gains):
        valid = sample.angle_valid
        values = [sample.angle, sample.target, sample.error, sample.accel_angle, sample.rate] if valid else [""]*5
        self.writer.writerow((self.source, round(elapsed, 4), sample.ms, sample.seq, experiment,
                              trial, group, phase, round(phase_s, 3), *values, sample.pulse,
                              sample.cap, sample.mode, int(sample.armed), int(sample.imu), int(valid),
                              int(sample.gravity), sample.fault, *gains, *sample.raw, int(sample.launching),
                              *(sample.terms or ("", "", "")), sample.dip))

    def add_trial(self, idx, experiment, group, gains, score, fault, seed=None, phase_at_fault=""):
        row = {"source": self.source, "trial": idx, "experiment": experiment, "group": group,
               "gains": list(gains), "score": score.total, "fault": fault, "phase_at_fault": phase_at_fault,
               **{k: v for k, v in asdict(score).items() if k != "total"}}
        if seed is not None:
            row["candidate_seed"] = seed
        self.trials.append(row)
        fields = ("source", "trial", "experiment", "group", "kp", "ki", "kd", "score", "fault",
                  "mae", "overshoot", "undershoot", "steady", "ripple", "pulse_change", "capped_pct",
                  "settle_up_s", "settle_down_s", "upper_stop", "candidate_seed", "phase_at_fault")
        with (self.path / "trials.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=fields, extrasaction="ignore")
            writer.writeheader()
            for item in self.trials:
                writer.writerow({**item, **dict(zip(("kp", "ki", "kd"), item["gains"]))})
        self.stream.flush()
        return row

    def event(self, elapsed, name, detail=""):
        event = {"source": self.source, "elapsed_s": round(elapsed, 3), "event": name, "detail": detail}
        self.events.append(event)
        with (self.path / "events.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=event)
            writer.writeheader()
            writer.writerows(self.events)

    def close(self):
        self.stream.close()


def propose(trials, ki, seed, selection, reference):
    model = Models(trials)
    candidate = model.shortlist(trials, {"Random", "Guided"}) if selection else model.next(seed, reference)
    return candidate, model.map(ki), model


class Runner:
    def __init__(self, rig, calibration, session):
        self.rig = rig
        self.calibration = calibration
        self.session = session
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.random = np.random.default_rng(29121)
        self.latest = None
        self.time = self.last_good = self.heartbeat = self.flush_due = 0.0
        self.gains = GAINS
        self.targets = (0., 20., 10.)
        self.experiment = "Monitor"
        self.phase = "Idle"
        self.phase_started = self.time
        self.phase_device_ms = None
        self.trial = 0
        self.group = ""
        self.scores = {}
        self.queue = []
        self.phase_index = 0
        self.future = None
        self.status = "Waiting for telemetry."
        self.last_fault = 0
        self.fit = None
        self.maps = None
        self.last_map_ki = None
        self.shortlist = []
        self.pick = None
        self.best_single = None
        self.baseline = None
        self.recommended = None
        self.confirmed_gain = None
        self.ramp_crossing = None
        self.ramp_pulse = None
        self.ramp_cap_since = None
        self.ramp_started_at = None
        self.ramp_start_angle = None
        self.noise_stats = None
        self.cool_until = 0
        self.seed = None
        self.preload_gains = GAINS
        self.last_dip = -15
        self.hold_goal = None
        self.hold_high_since = None
        self.next_target_update = 0.0
        self.last_target_update = -3.0
        self.demo_stage = None
        self.demo_since = 0.0
        self.demo_dip_seen = False
        self.demo_with_dip = True

    @property
    def active(self):
        return self.experiment != "Monitor"

    @property
    def ready(self):
        return (self.calibration is not None and self.latest is not None and self.latest.angle_valid
                and self.time-self.last_good < 0.25 and self.latest.cap == self.calibration.cap)

    def send(self, *commands):
        for command in commands:
            self.rig.send(command)

    def reset_phase(self, phase):
        self.phase, self.phase_started, self.phase_device_ms = phase, self.time, None

    def mark(self, name, detail=""):
        self.session.event(self.time, name, detail)

    def motor_idle(self, status=None):
        self.send("X")
        self.hold_goal = None
        self.hold_high_since = None
        self.demo_stage = None
        self.experiment = "Monitor"
        self.reset_phase("Idle")
        self.group = ""
        self.queue.clear()
        if self.future:
            self.future.cancel()
            self.future = None
        if status:
            self.status = status

    def stop(self):
        self.motor_idle("Stopped and disarmed. Confirm idle before touching the arm.")
        self.mark("STOP")

    def update_calibration(self, cal):
        if self.active or (self.latest and self.latest.armed):
            raise ValueError("Stop and disarm before applying calibration.")
        self.send(*cal.commands())
        self.calibration = cal
        self.session.metadata["calibration"] = asdict(cal)
        self.session.write_metadata()
        self.status = "Calibration sent. Verify the displayed angle at level and both stops."
        self.mark("Calibration applied")

    def arm(self):
        if not self.ready:
            raise ValueError("Wait for a valid calibrated angle and the configured pulse limit.")
        if not self.latest.gravity or self.latest.angle >= self.calibration.targets[1]:
            raise ValueError("Return the arm below the maximum target and wait for a stable angle.")
        self.send("H", "A")
        self.mark("Arm requested")
        self.status = "Arm requested. Confirm the Armed status before starting."

    def require_armed(self):
        if not self.ready or not self.latest.armed or self.latest.fault:
            raise ValueError("Arm the calibrated rig first.")
        if self.active:
            raise ValueError("Stop the current experiment before starting another.")

    def check_targets(self, targets):
        lo, hi = self.calibration.targets
        if len(targets) != 3 or not all(lo <= v <= hi for v in targets) or not targets[0] < targets[2] < targets[1]:
            raise ValueError(f"Use start < down < up and keep all targets inside {lo:g}° to {hi:g}°.")
        if self.session.trials and tuple(self.session.metadata["trial_targets_deg"]) != targets:
            raise ValueError("Use the same three targets for scored runs in this session.")
        self.targets = targets
        self.session.metadata["trial_targets_deg"] = list(targets)
        self.session.write_metadata()

    def gain_command(self, gains=None, command="G"):
        return command + " " + " ".join(f"{v:.5f}" for v in (self.gains if gains is None else gains))

    def start_hold(self, target, gains):
        self.require_armed()
        lo, hi = self.calibration.targets
        if not lo <= target <= hi:
            raise ValueError(f"Set a target inside {lo:g}° to {hi:g}°.")
        self.gains = gains
        self.hold_high_since = None
        self.experiment = "Hold"
        self.group = "PID" if gains[1] != 0 else ("PD" if gains[2] != 0 else "P")
        self.reset_phase("Hold")
        self.send("S", self.gain_command(), f"T {target:.2f}", "R")
        self.status = f"{self.group} hold at {target:g}°."
        self.mark("Hold started", f"{target:g}° · {self.group} · gains {gains}")

    @property
    def can_retarget(self):
        s = self.latest
        return bool(self.experiment == "Hold" and self.ready and s.armed and s.mode == 1
                    and not s.fault and not s.launching and s.dip in (0, None)
                    and self.time-self.last_dip >= 15)

    def change_target(self, target):
        if not self.can_retarget:
            raise ValueError("Wait for the hold to begin or for the disturbance recording to finish.")
        lo, hi = self.calibration.targets
        if not lo <= target <= hi:
            raise ValueError(f"Set a target inside {lo:g}° to {hi:g}°.")
        self.hold_goal = round(target, 2)
        self.status = f"{self.group} · moving target to {self.hold_goal:g}° at up to 2°/s."
        self.mark("Live target requested", f"{self.hold_goal:g}°")

    def start_demo(self, with_dip=True):
        self.require_armed()
        if self.latest.terms is None:
            raise ValueError("The timed demonstration requires the current Uno firmware.")
        lo, hi = self.calibration.targets
        if not lo <= DEMO_START < DEMO_NEXT <= hi:
            raise ValueError("The timed demonstration needs calibrated targets from −9° to −6°.")
        self.start_hold(DEMO_START, GAINS)
        self.demo_stage, self.demo_since = "Hold", self.time
        self.demo_dip_seen = False
        self.demo_with_dip = with_dip
        profile = "PID · −9° to −6°"
        self.mark("Demo started", profile + (" · one command dip" if with_dip else " · tracking only"))
        self.status = "Demo · hold at −9°."

    def advance_demo(self):
        elapsed = self.time-self.demo_since
        if self.latest.mode == 1 and (self.latest.angle > 5 or abs(self.latest.rate) > 110):
            self.mark("Demo incomplete", "Demonstration angle or rate limit reached")
            self.motor_idle("Demo stopped outside its motion window.")
            return
        if self.demo_stage == "Hold":
            if elapsed >= 12 and self.can_retarget and abs(self.latest.error) <= 3.5:
                self.change_target(DEMO_NEXT)
                self.demo_stage, self.demo_since = "Step", self.time
                self.status = "Demo · moving the target to −6°."
            elif elapsed >= 24:
                self.mark("Demo incomplete", "First hold did not qualify")
                self.motor_idle("Demo ended at the first hold. Use the recorded Results.")
        elif self.demo_stage == "Step":
            if (not self.demo_with_dip and elapsed >= 12 and self.can_retarget
                    and abs(self.latest.error) <= 3.5):
                self.mark("Demo complete", "Tracking only")
                self.motor_idle("Tracking demo complete. The motor is disarmed.")
            elif self.demo_with_dip and elapsed >= 12 and self.can_dip and abs(self.latest.error) <= 2.5:
                self.request_dip()
                self.demo_stage, self.demo_since = "Recovery", self.time
                self.status = "Demo · 2-second command dip and recovery."
            elif elapsed >= 24:
                reason = "Dip gate did not qualify" if self.demo_with_dip else "Second hold did not qualify"
                self.mark("Demo incomplete", reason)
                self.motor_idle(f"Demo ended. {reason}. Use the recorded Results.")
        elif self.demo_stage == "Recovery":
            self.demo_dip_seen |= self.latest.dip == -4
            if elapsed >= 1.2 and not self.demo_dip_seen:
                self.mark("Demo incomplete", "No applied dip in Uno telemetry")
                self.motor_idle("Demo ended. The Uno did not apply a dip.")
            elif elapsed >= 18:
                self.mark("Demo complete")
                self.motor_idle("Demo complete. The motor is disarmed.")

    @property
    def can_dip(self):
        s = self.latest
        return bool(self.experiment == "Hold" and self.ready and s.dip == 0 and s.armed and s.mode == 1
                    and not s.launching and abs(s.error) < 4.5 and abs(s.rate) < 6
                    and s.angle < self.calibration.firmware_upper-7 and self.time-self.last_dip >= 15
                    and self.hold_goal is None and self.time-self.last_target_update >= 3)

    def request_dip(self):
        if not self.can_dip:
            raise ValueError("Wait for the arm to hold near the target before applying the pulse dip.")
        self.send("D")
        self.last_dip = self.time
        self.mark("Pulse dip requested", f"{self.group} · command −4 µs for 2 s")

    def start_noise(self):
        if self.active or (self.latest and self.latest.armed):
            raise ValueError("Stop and disarm before the motor-off noise test.")
        if not self.ready:
            raise ValueError("Apply the three-pose calibration before testing tilt noise.")
        self.experiment = "Noise"
        self.group = "Motor off"
        self.reset_phase("Noise")
        self.scores = {"Noise": []}
        self.status = "Recording 4 seconds of stationary MPU6050 tilt."
        self.mark("Noise started")

    def start_ramp(self):
        self.require_armed()
        self.experiment = "Ramp"
        self.group = "Open loop"
        self.reset_phase("Ramp")
        self.ramp_pulse = max(1110, min(1127, self.calibration.hover-15))
        self.ramp_crossing = None
        self.ramp_cap_since = None
        self.ramp_started_at = self.time
        self.ramp_start_angle = self.latest.angle
        self.send(f"P {self.ramp_pulse}")
        self.status = "Searching for level in 1 µs steps. The sweep pauses while the arm climbs."
        self.mark("Ramp started")

    def start_baseline(self, gains, targets):
        self.require_armed()
        self.check_targets(targets)
        if self.baseline is not None and self.baseline != gains:
            raise ValueError("Keep the original manual PID gains for the baseline in this session.")
        self.baseline = gains
        self.experiment = "Baseline"
        self.queue = self.interleave([("Baseline", gains)], "Baseline", 5)
        if not self.queue:
            self.experiment = "Monitor"
            raise ValueError("The five baseline runs are already complete.")
        self.mark("Baseline started")
        self.next_trial()

    def start_hbo(self, targets):
        self.require_armed()
        if self.latest.terms is None:
            raise ValueError("Upload the updated firmware for the shared PID preload and HBO trials.")
        self.check_targets(targets)
        if self.pick:
            raise ValueError("This session already has an HBO pick. Reconnect for a fresh session.")
        if len(self.baseline_trials()) < 5:
            raise ValueError("Complete five manual baseline trials at these targets first.")
        if not np.all((BOUNDS[:, 0] <= self.baseline) & (self.baseline <= BOUNDS[:, 1])):
            raise ValueError("Use manual PID gains inside the tuning range shown on the score maps.")
        self.session.metadata["tuning_protocol"] = "Baseline anchors and interleaved confirmation"
        self.session.write_metadata()
        self.experiment = "HBO"
        self.mark("HBO started")
        self.next_trial()

    def start_repeat(self, targets, baseline):
        self.require_armed()
        self.check_targets(targets)
        if not self.pick or not self.best_single:
            raise ValueError("Complete HBO before running the independent repeat comparison.")
        self.experiment = "Repeat"
        # Interleave groups to reduce drift bias, especially motor warming.
        groups = [("Baseline", self.baseline or baseline),
                  ("Best single", self.best_single), ("HBO pick", self.pick)]
        self.queue = self.interleave(groups, "Repeat", 5)
        if not self.queue:
            self.experiment = "Monitor"
            raise ValueError("The five independent repeats per method are already complete.")
        self.mark("Repeat started")
        self.next_trial()

    def start_matched_hbo(self, comparison):
        self.require_armed()
        if self.latest.terms is None:
            raise ValueError("Use the current Uno firmware with bumpless gain handover.")
        if self.calibration.hover != comparison.trim:
            raise ValueError(f"This comparison requires a shared trim of {comparison.trim} µs.")
        existing = self.session.metadata.get("matched_hbo")
        settings = asdict(comparison)
        if existing is not None and existing["settings"] != settings:
            raise ValueError("Reconnect before changing the candidate or comparison settings.")
        if self.session.trials and existing is None:
            raise ValueError("Reconnect for a fresh session before starting the paired comparison.")
        self.check_targets(comparison.targets)
        groups = [("Baseline", comparison.baseline), ("HBO pick", comparison.candidate)]
        counts = [len(self.completed_scores(MATCHED_HBO, group)) for group, _ in groups]
        # Reverse the order of each second pair to reduce systematic order effects.
        queue = [groups[i] for pair in range(MATCHED_PAIRS)
                 for i in ((1, 0) if pair % 2 else (0, 1)) if counts[i] <= pair]
        if not queue:
            raise ValueError("The five pairs are already complete. Open Results to review them.")
        self.session.metadata["matched_hbo"] = existing or {
            "settings": settings,
            "planned_pairs": MATCHED_PAIRS,
            "required_mean_reduction": 0.10,
            "required_pair_wins": MATCHED_PAIRS // 2 + 1,
        }
        self.session.write_metadata()
        self.experiment = MATCHED_HBO
        self.queue = queue
        self.mark("Matched HBO started", f"Shared trim {comparison.trim} µs")
        self.next_trial()

    def next_trial(self):
        self.seed = None
        if self.queue:
            group, gains = self.queue.pop(0)
            self.begin_trial(group, gains)
            return
        if self.experiment != "HBO":
            if self.experiment == "Repeat":
                self.complete_validation()
            if self.experiment == MATCHED_HBO:
                self.complete_matched_hbo()
            else:
                self.motor_idle(f"{self.experiment} complete. Saved to {self.session.path.name}.")
            self.mark("Experiment complete")
            return
        search = self.search_trials()
        retests = [row for row in self.session.trials if row["experiment"] == "HBO" and row["group"] == "Shortlist"
                   and not row["fault"]]
        if self.shortlist:
            groups = [("Confirmation baseline", self.baseline),
                      *(("Shortlist", g) for g in self.shortlist)]
            self.queue = self.interleave(groups, "HBO", 3)
            if self.queue:
                self.next_trial()
                return
            control = self.completed_scores("HBO", "Confirmation baseline")
            choices = [(np.mean([r["score"] for r in retests if r["gains"] == list(g)]), g) for g in self.shortlist]
            self.pick = min(choices, key=lambda item: item[0])[1]
            candidate_scores = [r["score"] for r in retests if r["gains"] == list(self.pick)]
            confirmed = self.beats_baseline(candidate_scores, control)
            self.confirmed_gain = self.pick if confirmed else self.baseline
            self.recommended = self.baseline
            self.session.metadata["hbo_pick"] = [float(x) for x in self.pick]
            self.session.metadata["best_single"] = [float(x) for x in self.best_single]
            self.session.metadata["confirmation"] = {"baseline": control, "candidate": candidate_scores,
                                                     "improved": confirmed}
            self.session.metadata["recommended_gains"] = list(self.recommended)
            self.session.write_metadata()
            self.mark("HBO pick", str(tuple(round(v, 5) for v in self.pick)))
            self.motor_idle("HBO complete. Run independent validation to compare against manual PID.")
            if self.fit:
                self.maps = self.fit.map(self.pick[1])
                self.last_map_ki = self.pick[1]
            return
        if len(search) < 8:
            self.seed = int(self.random.integers(2**31))
            random = np.random.default_rng(self.seed)
            point = random.random(3) if len(search) >= 4 else np.clip(
                unit(self.baseline) + random.normal(0, 0.14, 3), 0, 1)
            self.begin_trial("Random", tuple(gains_at(point)))
            return
        if len(search) >= 30:
            valid = [row for row in search if not row["fault"]]
            if not valid:
                self.motor_idle("No complete safe search trials. Check the rig and calibration.")
                return
            self.best_single = tuple(min(valid, key=lambda row: row["score"])["gains"])
        self.seed = int(self.random.integers(2**31))
        self.future = self.pool.submit(propose, self.baseline_trials()+search, self.gains[1],
                                       self.seed, len(search) >= 30, self.baseline)
        self.reset_phase("Selecting")
        self.status = "Fitting the score and noise models while the motor rests."

    def search_trials(self):
        return [r for r in self.session.trials if r["experiment"] == "HBO" and r["group"] in ("Random", "Guided")
                and np.isfinite(r["score"])]

    def baseline_trials(self):
        return [r for r in self.session.trials if r["experiment"] == "Baseline" and not r["fault"]]

    def completed_scores(self, experiment, group):
        return [r["score"] for r in self.session.trials
                if r["experiment"] == experiment and r["group"] == group and not r["fault"]]

    def interleave(self, groups, experiment, repeats):
        pending = []
        for group, gains in groups:
            completed = sum(r["experiment"] == experiment and r["group"] == group
                            and r["gains"] == list(gains) and not r["fault"] for r in self.session.trials)
            pending.append((group, gains, completed))
        return [(group, gains) for round_index in range(repeats)
                for group, gains, count in pending if count <= round_index]

    @staticmethod
    def beats_baseline(candidate, baseline):
        return bool(len(candidate) == len(baseline) and len(baseline) >= 3
                    and np.mean(candidate) <= 0.90*np.mean(baseline)
                    and sum(test < control for test, control in zip(candidate, baseline)) > len(baseline)//2)

    def complete_validation(self):
        control = self.completed_scores("Repeat", "Baseline")
        candidate = self.completed_scores("Repeat", "HBO pick")
        candidate_tripped = any(r["experiment"] == "Repeat" and r["group"] == "HBO pick"
                                and r["fault"] and r["phase_at_fault"] in ("Up", "Down")
                                for r in self.session.trials)
        improved = (self.confirmed_gain == self.pick and not candidate_tripped
                    and self.beats_baseline(candidate, control))
        self.recommended = self.pick if improved else self.baseline
        self.session.metadata["validation"] = {"baseline": control, "candidate": candidate, "improved": improved}
        self.session.metadata["recommended_gains"] = list(self.recommended)
        self.session.write_metadata()
        self.mark("Validation verdict", "HBO improved tracking" if improved else "Keep manual PID")

    def complete_matched_hbo(self):
        baseline = self.completed_scores(MATCHED_HBO, "Baseline")
        candidate = self.completed_scores(MATCHED_HBO, "HBO pick")
        scored_faults = sum(row["experiment"] == MATCHED_HBO and bool(row["fault"])
                            and row["phase_at_fault"] in ("Up", "Down") for row in self.session.trials)
        reduction = 1 - float(np.mean(candidate)) / float(np.mean(baseline))
        wins = sum(test < control for test, control in zip(candidate, baseline))
        improved = not scored_faults and self.beats_baseline(candidate, baseline)
        self.session.metadata["matched_hbo"]["result"] = {
            "baseline": baseline, "candidate": candidate, "mean_reduction": reduction,
            "candidate_wins": wins, "scored_faults": scored_faults, "improved": improved,
        }
        self.session.write_metadata()
        status = ("Scored-stage fault. Improvement criterion was not met." if scored_faults else
                  "HBO met the paired improvement criterion." if improved else
                  "HBO did not meet the paired improvement criterion.")
        self.mark("Matched HBO verdict", status)
        self.motor_idle(f"{status} Open Results for the scores.")

    def begin_trial(self, group, gains):
        self.send("S")
        self.trial += 1
        self.group = group
        self.gains = tuple(round(float(v), 5) for v in gains)
        self.phase_index = 0
        self.scores = {name: [] for name, _ in TRIAL_PHASES}
        self.reset_phase("Rest")
        self.status = f"{self.experiment} · trial {self.trial} · {group} · Rest"

    def transition(self):
        phase = self.phase_index + 1
        if phase >= len(TRIAL_PHASES):
            self.send("S")
            try:
                score = score_trial(self.scores)
            except ValueError as exc:
                self.motor_idle(str(exc))
                self.mark("Invalid trial", str(exc))
                return
            self.session.add_trial(self.trial, self.experiment, self.group, self.gains, score, 0, self.seed)
            self.mark("Trial complete", f"{self.trial}: {score.total:.3f}")
            if self.experiment in ("HBO", "Repeat") and len(self.session.trials) % 10 == 0:
                self.cool_until = self.time + 15
                self.status = "Cooling pause · motor off for 15 seconds."
                self.reset_phase("Cooling")
                return
            self.next_trial()
            return
        self.phase_index = phase
        self.reset_phase(TRIAL_PHASES[phase][0])
        target = self.targets[phase-1]
        if phase == 1:
            gains = self.preload_gains if self.latest.terms is not None else self.gains
            self.send(self.gain_command(gains), f"T {target:.2f}", "R")
        else:
            self.send(f"T {target:.2f}")
        self.status = f"{self.experiment} · trial {self.trial} · {self.group} · {self.phase}"

    def handle_fault(self, sample):
        if sample.fault != self.last_fault and sample.fault:
            name = FAULTS[sample.fault]
            if self.experiment in SCORED_EXPERIMENTS:
                if self.phase in ("Level", "Set gains", "Up", "Down"):
                    scored_cutoff = sample.fault == 4 and self.phase in ("Up", "Down")
                    score = score_trial(self.scores, True) if scored_cutoff else Score(
                        *([float("nan")]*10), sample.fault == 4)
                    self.session.add_trial(self.trial, self.experiment, self.group, self.gains,
                                           score, sample.fault, self.seed, self.phase)
                self.mark("Trial aborted", f"{self.trial} · {self.group} · {self.phase} · {name}")
            self.mark("Safety cutoff", name)
            self.motor_idle(f"Safety cutoff · {name}. Inspect the rig before arming again.")
        self.last_fault = sample.fault

    def tick(self, dt):
        self.time += dt
        if self.time >= self.heartbeat:
            self.send("H")
            self.heartbeat = self.time + 0.18
        rows = self.rig.poll(dt)
        for sample in rows:
            self.latest, self.last_good = sample, self.time
            self.handle_fault(sample)
            if self.experiment == "Hold" and sample.mode == 1 and sample.angle_valid:
                excess = sample.angle - sample.target
                if excess < HOLD_OVERSHOOT_DEG:
                    self.hold_high_since = None
                elif self.hold_high_since is None:
                    self.hold_high_since = sample.ms
                elif sample.ms - self.hold_high_since >= HOLD_OVERSHOOT_MS:
                    detail = f"Target {sample.target:+.1f}°, measured {sample.angle:+.1f}°"
                    self.motor_idle(f"Hold stopped on sustained overshoot. {detail}.")
                    self.mark("Hold overshoot", detail)
            else:
                self.hold_high_since = None
            expected_mode = 0 if self.phase in ("Rest", "Noise") else 1
            expected_target = self.targets[self.phase_index-1] if self.phase in ("Level", "Up", "Down") else None
            matches = sample.mode == expected_mode and (expected_target is None or abs(sample.target-expected_target) <= 0.025)
            record_score = self.phase in self.scores and not sample.launching
            if (record_score or self.phase == "Hold") and matches and sample.angle_valid:
                if self.phase_device_ms is None:
                    self.phase_device_ms = sample.ms
                if record_score:
                    self.scores[self.phase].append(sample)
            elapsed = max(0, (sample.ms-self.phase_device_ms)/1000) if self.phase_device_ms is not None else 0
            logged_gains = (self.preload_gains if self.phase in ("Level", "Set gains") and self.active
                            and sample.terms is not None else self.gains)
            self.session.log(self.time, sample, self.experiment,
                             self.trial if self.experiment in SCORED_EXPERIMENTS else 0,
                             self.group, self.phase, elapsed, logged_gains)
        acknowledgements = self.rig.acks[:]
        self.rig.acks.clear()
        for cmd, ok in acknowledgements:
            if cmd == "D":
                self.mark("Pulse dip accepted" if ok else "Pulse dip rejected")
                if not ok:
                    self.status = "Pulse dip skipped. Wait for a steady angle and try again."
                continue
            if cmd == "J" and ok and self.phase == "Set gains":
                self.transition()
                continue
            if not ok and cmd not in ("H", "X"):
                self.motor_idle(f"The Uno refused {cmd}. Check calibration, target and arm status.")
                self.mark("Command rejected", cmd)
                break
        if self.active and self.time - self.last_good > 0.38:
            self.motor_idle("Telemetry stopped. The Uno's watchdog returns it to idle.")
        if self.time >= self.flush_due:
            self.session.stream.flush()
            self.flush_due = self.time + 0.7
        self.advance()
        if self.demo_stage is not None:
            self.advance_demo()
        return rows

    def advance(self):
        if self.phase == "Selecting" and self.future and self.future.done():
            future, self.future = self.future, None
            candidate, self.maps, self.fit = future.result()
            self.last_map_ki = self.gains[1]
            if len(self.search_trials()) >= 30:
                self.shortlist = [tuple(round(float(v), 5) for v in gains) for gains in candidate]
                if not candidate:
                    self.motor_idle("HBO found no valid candidates to verify.")
                    return
                self.next_trial()
            else:
                self.begin_trial("Guided", candidate)
        elif self.phase == "Cooling" and self.time >= self.cool_until:
            self.next_trial()
        elif self.phase == "Noise":
            if self.time-self.phase_started >= 4:
                data = [s.accel_angle for s in self.scores["Noise"]]
                tilt = [s.angle for s in self.scores["Noise"]]
                if len(data) < 130:
                    self.motor_idle("Noise capture incomplete. Check the IMU.")
                else:
                    self.noise_stats = float(np.std(data)), float(np.std(tilt))
                    self.mark("Noise result", str(self.noise_stats))
                    self.motor_idle(f"IMU standard deviation · accel {self.noise_stats[0]:.2f}° · filtered {self.noise_stats[1]:.2f}°")
        elif self.phase == "Ramp":
            if self.latest and self.latest.angle_valid and self.latest.mode == 2 and self.latest.angle >= 0:
                self.ramp_crossing = self.latest.pulse
                self.mark("Level crossing", str(self.ramp_crossing))
                self.motor_idle(f"First level crossing at {self.ramp_crossing} µs. Check trim and available headroom.")
            elif self.time-self.ramp_started_at >= 40:
                self.motor_idle("Level search timed out. Inspect the lift and balance before retrying.")
                self.mark("Ramp timed out")
            elif self.time-self.phase_started >= 0.55:
                if self.ramp_pulse >= self.calibration.cap:
                    if self.ramp_cap_since is None:
                        self.ramp_cap_since = self.time
                    elif self.time-self.ramp_cap_since >= 1.7:
                        self.motor_idle("Level not reached within the ESC command range. Check balance and thrust.")
                        self.mark("Ramp ended at cap")
                elif (self.ramp_pulse >= self.calibration.hover+15 and self.latest
                      and self.latest.angle < self.ramp_start_angle+3):
                    self.motor_idle("No clear lift near the expected hover pulse. Check the motor and balance.")
                    self.mark("Ramp stopped without lift")
                elif self.latest and self.latest.rate >= 2:
                    self.phase_started = self.time
                else:
                    self.ramp_pulse += 1
                    self.send(f"P {self.ramp_pulse}")
                    self.phase_started = self.time
        elif self.phase == "Hold":
            if self.time-self.phase_started > 1.2 and (not self.latest or self.latest.mode != 1):
                self.motor_idle("The Uno did not enter PID hold.")
            elif self.hold_goal is not None and self.can_retarget and self.time >= self.next_target_update:
                current = self.latest.target
                delta = self.hold_goal-current
                if abs(delta) <= .01:
                    self.mark("Live target applied", f"{current:g}°")
                    self.status = f"{self.group} hold at {current:g}°."
                    self.hold_goal = None
                else:
                    target = current + max(-LIVE_TARGET_STEP, min(LIVE_TARGET_STEP, delta))
                    self.send(f"T {target:.2f}")
                    self.last_target_update = self.time
                    self.next_target_update = self.time + LIVE_TARGET_INTERVAL
        elif self.phase == "Set gains" and self.time-self.phase_started > .7:
            self.mark("Trial aborted", f"{self.trial} · {self.group} · Gains handover timed out")
            self.motor_idle("The Uno did not confirm the trial gains.")
        elif self.phase in dict(TRIAL_PHASES) and self.experiment in SCORED_EXPERIMENTS:
            waiting_for_launch = self.phase == "Level" and self.latest and self.latest.launching
            if self.phase_device_ms is None and self.time-self.phase_started > 1.2 and not waiting_for_launch:
                self.motor_idle("The board did not enter the expected trial phase.")
            elif self.phase_device_ms is not None and self.latest and (
                    self.latest.ms-self.phase_device_ms >= 1000*dict(TRIAL_PHASES)[self.phase]):
                if self.phase == "Level":
                    recent = self.scores["Level"][-30:]
                    if (not recent or np.mean([abs(s.error) for s in recent]) > 2.0
                            or abs(self.latest.error) > 2.5 or abs(self.latest.rate) > 8):
                        if self.latest.ms-self.phase_device_ms >= 14000:
                            self.mark("Trial aborted", f"{self.trial} · {self.group} · Level did not settle")
                            self.motor_idle("Start angle did not settle within 2°. Re-arm to retry this trial.")
                        return
                    if self.latest.terms is not None and (
                            self.gains != self.preload_gains or self.experiment == MATCHED_HBO):
                        self.reset_phase("Set gains")
                        self.send(self.gain_command(command="J"))
                        return
                self.transition()

    def close(self):
        try:
            self.rig.close()
        finally:
            self.pool.shutdown(wait=False, cancel_futures=True)
            self.session.close()
