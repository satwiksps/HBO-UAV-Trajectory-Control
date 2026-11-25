"""Summaries and disturbance metrics from recorded, time-stamped samples."""
from collections import defaultdict

import numpy as np


def trial_groups(trials, metadata):
    """Keep tuning confirmation and independent validation in distinct groups."""
    groups = defaultdict(list)
    pick = metadata.get("hbo_pick")
    for row in trials:
        experiment, group = row["experiment"], row["group"]
        if experiment == "Baseline":
            groups["Manual baseline"].append(row)
        elif experiment == "Matched HBO":
            label = "Paired · manual PID" if group == "Baseline" else "Paired · HBO candidate"
            groups[label].append(row)
        elif experiment == "Repeat":
            groups[f"Validation · {group}"].append(row)
        elif group == "Confirmation baseline":
            groups["Confirmation · manual"].append(row)
        elif group == "Shortlist" and pick:
            gains = row.get("gains") or [float(row[k]) for k in ("kp", "ki", "kd")]
            if np.allclose(gains, pick):
                groups["Confirmation · HBO pick"].append(row)
    return dict(groups)


def valid_trials(group):
    return [row for row in group if int(row["fault"]) == 0 and np.isfinite(float(row["score"]))]


def mean_field(rows, field):
    values = np.array([float(row[field]) for row in rows], dtype=float)
    finite = values[np.isfinite(values)]
    return float(finite.mean()) if finite.size else None


def matched_holds(rows, events):
    """Measure the same pre-disturbance interval in each completed matched hold."""
    start = None
    number = 0
    for event in events:
        if event["event"] == "Matched test":
            start = float(event["elapsed_s"])
            number += 1
        elif start is not None and event["event"] == "Matched test result":
            end = float(event["elapsed_s"])
            hold = [r for r in rows if r["experiment"] == "Hold" and r["mode"] == "1"
                    and r["angle_valid"] == "1" and start <= float(r["elapsed_s"]) <= end]
            if hold:
                elapsed = np.array([float(r["elapsed_s"])-start for r in hold])
                angle = np.array([float(r["angle_deg"]) for r in hold])
                target = float(hold[0]["target_deg"])
                fixed = (elapsed >= 12) & (elapsed < 18)
                if fixed.sum() >= 200:
                    yield {"trial": number, "controller": hold[0]["group"], "target": target, "t": elapsed,
                           "angle": angle, "mae": float(np.mean(np.abs(angle[fixed]-target))),
                           "bias": float(np.mean(angle[fixed]-target)), "ripple": float(np.std(angle[fixed])),
                           "peak_angle": float(np.max(angle)), "start_s": start}
            start = None
        elif event["event"] in ("Safety cutoff", "Supervisor abort", "STOP"):
            start = None


def dip_windows(rows):
    """Use the offset reported by the board, with device time for recovery metrics."""
    start = None
    for i, row in enumerate(rows):
        active = row.get("disturbance_us") == "-4" and row["mode"] == "1"
        if active and start is None:
            start = i
        if active or start is None:
            continue
        pulse_start, pulse_end = int(rows[start]["device_ms"]), int(row["device_ms"])
        first = rows[start]
        if not 1700 <= pulse_end-pulse_start <= 2200 or row["mode"] != "1":
            start = None
            continue
        lo = start
        while lo and pulse_start-int(rows[lo-1]["device_ms"]) <= 2000 and (
                rows[lo-1]["experiment"], rows[lo-1]["mode"], rows[lo-1]["group"]) == ("Hold", "1", first["group"]):
            lo -= 1
        hi = i
        while hi < len(rows) and int(rows[hi]["device_ms"])-pulse_end <= 30000 and (
                rows[hi]["experiment"], rows[hi]["mode"], rows[hi]["group"], rows[hi]["target_deg"],
                rows[hi].get("disturbance_us")) == ("Hold", "1", first["group"], first["target_deg"], "0"):
            hi += 1
        window = [r for r in rows[lo:hi] if r["angle_deg"] and r["angle_valid"] == "1"]
        if window and first["experiment"] == "Hold":
            t = (np.array([int(r["device_ms"]) for r in window])-pulse_start)/1000
            y = np.array([float(r["angle_deg"]) for r in window])
            target = float(first["target_deg"])
            removal = (pulse_end-pulse_start)/1000
            measured = t >= 0
            error = np.abs(y-target)
            after = np.flatnonzero(t >= removal)
            if not np.any(measured) or not np.any(t < 0):
                start = None
                continue
            before = float(y[t < 0].mean())
            def recovery_to(reference):
                for index in after:
                    end = np.searchsorted(t, t[index]+1.0)
                    if end < len(t) and np.all(np.abs(y[index:end+1]-reference) <= 1.0) and np.diff(t[index:end+1]).max() < .25:
                        return float(t[index]-removal)
                return None
            yield {"rows": window, "t": t, "angle": y, "target": target, "removal": removal,
                   "group": first["group"], "peak": float(error[measured].max()),
                   "pre_error": abs(before-target), "recovery": recovery_to(target),
                   "equilibrium_recovery": recovery_to(before),
                   "elapsed_s": first["elapsed_s"], "gains": [float(first[k]) for k in ("kp", "ki", "kd")]}
        start = None
