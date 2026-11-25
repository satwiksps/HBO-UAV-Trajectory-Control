"""Export figures from a single saved session. Empty experiments are skipped."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from hbo import BOUNDS, Models
from results import dip_windows, matched_holds, trial_groups, valid_trials

INK = "#182832"
TEAL = "#0c8582"
ORANGE = "#bd672f"
MUTED = "#607683"
PALETTE = {"Baseline": "#536a85", "Best single": ORANGE, "HBO pick": TEAL}


def read_csv(path):
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as file:
        return list(csv.DictReader(file))


def series(rows, key):
    return np.array([float(r[key]) if r[key] else np.nan for r in rows])


def frame(title, subtitle, nrows=1, ncols=1, figsize=None):
    figure, axes = plt.subplots(nrows, ncols, figsize=figsize or (8, 4.6), squeeze=False)
    figure.set_facecolor("#ffffff")
    figure.text(.10, .957, title, color=INK, fontsize=16, weight="bold", va="top")
    figure.text(.10, .896, subtitle, color=MUTED, fontsize=9, va="top")
    figure.subplots_adjust(top=.80, bottom=.14, left=.10, right=.92, hspace=.48, wspace=.40)
    for ax in axes.ravel():
        ax.set_axisbelow(True)
        ax.grid(color="#e0e8ed", alpha=.85, linewidth=.75)
        ax.tick_params(colors=INK, labelsize=9, length=0)
        ax.xaxis.label.set_color(INK)
        ax.yaxis.label.set_color(INK)
        for key in ("top", "right"):
            ax.spines[key].set_visible(False)
        for key in ("left", "bottom"):
            ax.spines[key].set_color("#bccad2")
    return figure, axes


def export(path):
    path = Path(path).resolve()
    meta = json.loads((path / "session.json").read_text(encoding="utf-8"))
    data = read_csv(path / "telemetry.csv")
    attempts = read_csv(path / "trials.csv")
    events = read_csv(path / "events.csv")
    source = meta["source"]
    if source not in ("SIMULATION", "HARDWARE") or any(
            row["source"] != source for row in data + attempts + events):
        raise ValueError("Session contains conflicting or unknown data provenance.")
    out = path / "plots"
    out.mkdir(exist_ok=True)
    saved = []

    def save(figure, name):
        figure.text(.10, .028, source + "  /  " + path.name,
                    fontsize=8, color=ORANGE if source == "SIMULATION" else MUTED)
        if source == "SIMULATION":
            figure.text(.90, .945, "SIMULATION", color=ORANGE, fontsize=9, weight="bold", ha="right",
                        bbox={"facecolor": "#fff0e3", "edgecolor": "none", "pad": 5})
        target = out / f"{name}.png"
        figure.savefig(target, dpi=190, facecolor=figure.get_facecolor())
        plt.close(figure)
        saved.append(target)

    ramp = [r for r in data if r["experiment"] == "Ramp" and r["mode"] == "2" and r["angle_deg"]]
    if ramp:
        fig, axs = frame("Open-loop throttle ramp", "Angle during the bounded pulse ramp. Each point is a recorded sample.",
                         2, figsize=(8, 5.4))
        t = series(ramp, "elapsed_s")
        t -= t[0]
        ax, bx = axs[:, 0]
        ax.plot(t, series(ramp, "angle_deg"), color=TEAL, linewidth=1.6)
        ax.axhline(0, color=MUTED, ls="--", linewidth=0.8)
        ax.set(ylabel="Angle [°]")
        bx.step(t, series(ramp, "esc_us"), color=ORANGE, where="post")
        bx.set(xlabel="Ramp time [s]", ylabel="ESC [µs]")
        save(fig, "01_open_loop_ramp")

    scores = {int(row["trial"]): row for row in attempts if row["fault"] == "0"}
    if scores:
        paired = "matched_hbo" in meta
        experiment = "Matched HBO" if paired else "Repeat"
        selected = {name: [idx for idx, r in scores.items() if r["experiment"] == experiment and r["group"] == name]
                    for name in PALETTE}
        independent = bool(selected["Baseline"] and selected["HBO pick"])
        if not independent and not paired:
            selected = {
                "Baseline": [idx for idx, r in scores.items() if r["group"] == "Confirmation baseline"] or
                            [idx for idx, r in scores.items() if r["experiment"] == "Baseline"],
                "HBO pick": [idx for idx, r in scores.items() if r["group"] == "Shortlist" and "hbo_pick" in meta and
                             np.allclose([float(r[k]) for k in ("kp","ki","kd")], meta["hbo_pick"])],
            }
        subtitle = ("Paired validation at a shared trim · median and range across completed runs." if paired else
                    "Independent validation · median and range across completed runs." if independent else
                    "Preliminary comparison · median and range across completed baseline and shortlist runs.")
        fig, axs = frame("Closed-loop step response", subtitle, 1, 2, (10, 4.6))
        by_phase = {}
        for row in data:
            if row["phase"] in ("Up", "Down") and row["mode"] == "1" and row["angle_deg"] and float(row["phase_s"]) > 0:
                by_phase.setdefault((row["trial"], row["phase"]), []).append(row)
        drawn = False
        for ax, phase in zip(axs[0], ("Up", "Down")):
            matched = {}
            grid = np.linspace(0, 5.8, 160)
            for name, ids in selected.items():
                traces, targets = [], []
                for idx in ids:
                    trial = by_phase.get((str(idx), phase), [])
                    if len(trial) >= 120:
                        traces.append(np.interp(grid, series(trial, "phase_s"), series(trial, "angle_deg")))
                        targets.append(float(trial[-1]["target_deg"]))
                if traces and np.ptp(targets) < 0.05:
                    matched[name] = (np.array(traces), targets[0])
            if len(matched) == 2 and abs(matched["Baseline"][1]-matched["HBO pick"][1]) >= .05:
                matched.pop("HBO pick")
            for name, (traces, target) in matched.items():
                ax.plot(grid, np.median(traces, axis=0), color=PALETTE[name], label=f"{name} (n={len(traces)})",
                        linewidth=1.8)
                ax.fill_between(grid, traces.min(axis=0), traces.max(axis=0), color=PALETTE[name], alpha=.13)
                drawn = True
            if matched:
                ax.axhline(next(iter(matched.values()))[1], ls="--", color=MUTED, label="Target", linewidth=.85)
                ax.legend(frameon=False, fontsize=8)
            ax.set(xlabel="Time from command [s]", ylabel="Arm angle [°]", title=f"{phase} step")
        if drawn:
            save(fig, "02_step_response")
        else:
            plt.close(fig)

    search = [r for r in attempts if r["experiment"] == "HBO" and r["group"] in ("Random", "Guided")
              and np.isfinite(float(r["score"]))]
    if search:
        fig, axs = frame("Bayesian tuning progress", "Lower scores indicate better tracking. Crosses identify aborted upper-stop trials.")
        ax = axs[0, 0]
        x = np.arange(1, len(search)+1)
        y = series(search, "score")
        for label, color, fill in (("Random", MUTED, "none"), ("Guided", TEAL, TEAL)):
            use = [i for i, r in enumerate(search) if r["group"] == label and r["fault"] == "0"]
            if use:
                ax.scatter(x[use], y[use], edgecolor=color, facecolor=fill, label=label, s=35, zorder=4)
        failures = [i for i, r in enumerate(search) if r["fault"] != "0"]
        if failures:
            ax.scatter(x[failures], y[failures], marker="x", color=ORANGE, label="Upper-stop guard")
        best = np.minimum.accumulate(np.where([r["fault"] == "0" for r in search], y, np.inf))
        ax.step(x, np.where(np.isfinite(best), best, np.nan), where="post", color=INK, label="Best completed")
        baseline = [float(r["score"]) for r in attempts if r["experiment"] == "Baseline" and r["fault"] == "0"]
        if baseline:
            ax.axhline(np.mean(baseline), color=ORANGE, ls="--", lw=1.3, label="Manual baseline mean")
        ax.set(xlabel="Search trial", ylabel="Tracking score")
        ax.legend(frameon=False, fontsize=9)
        save(fig, "03_tuning_progress")

    if len(search) >= 8:
        anchors = (valid_trials(trial_groups(attempts, meta).get("Manual baseline", []))
                   if "tuning_protocol" in meta else [])
        train = [{**r, "gains": tuple(float(r[k]) for k in ("kp", "ki", "kd")), "score": float(r["score"])}
                 for r in anchors + search]
        chosen = np.array(meta.get("hbo_pick", train[int(np.argmin([r["score"] for r in train]))]["gains"]))
        model = Models(train)
        score, noise = model.map(chosen[1])
        x = np.geomspace(*BOUNDS[0], score.shape[0])
        y = np.geomspace(*BOUNDS[2], score.shape[1])
        for z, name, title, colorbar in ((score, "04_score_map", "Predicted tracking score", "Predicted score"),
                                        (noise, "05_noise_map", "Estimated trial variability", "Residual scale [log score]")):
            fig, axs = frame(title, f"Gaussian process slice at Ki = {chosen[1]:.4f}. Dots show projected search gains.")
            ax = axs[0, 0]
            layers = ax.contourf(x, y, z.T, levels=24, cmap="viridis_r")
            ax.set(xscale="log", yscale="log", xlabel="Kp [µs/°]", ylabel="Kd [µs·s/°]")
            points = np.array([r["gains"] for r in train])
            ax.scatter(points[:,0], points[:,2], s=12, facecolors="#f9fafb", edgecolors=INK, linewidth=.4)
            if "hbo_pick" in meta:
                ax.scatter(chosen[0], chosen[2], marker="*", s=140, color="#ff9e3f", edgecolors=INK,
                           zorder=5, label="Retested pick")
                ax.legend(loc="upper right", frameon=False, labelcolor="white", fontsize=8)
            fig.colorbar(layers, ax=ax, label=colorbar)
            save(fig, name)

    repeats = {name: [float(r["score"]) for r in attempts if r["experiment"] == "Repeat"
                      and r["group"] == name and r["fault"] == "0"] for name in PALETTE}
    groups = [(name, vals) for name, vals in repeats.items() if vals]
    if groups:
        fig, axs = frame("Repeatability after tuning", "Each dot is one independent repeat. Horizontal lines mark group medians.")
        ax = axs[0, 0]
        for x, (name, values) in enumerate(groups):
            offsets = np.linspace(-.09, .09, len(values))
            ax.scatter(x+offsets, values, color=PALETTE[name], s=42, zorder=4)
            ax.hlines(np.median(values), x-.21, x+.21, color=INK, linewidth=1.6)
        ax.set(xticks=range(len(groups)), xticklabels=[f"{n}\nn = {len(v)}" for n,v in groups],
               ylabel="Tracking score")
        save(fig, "06_repeatability")

    if result := meta.get("matched_hbo", {}).get("result"):
        baseline, candidate = result["baseline"], result["candidate"]
        trim = meta["matched_hbo"]["settings"]["trim"]
        fig, axs = frame("Manual PID and HBO candidate", f"Matched trials at {trim} µs. Lower score is better.")
        ax = axs[0, 0]
        pairs = range(1, len(baseline)+1)
        ax.plot(pairs, baseline, "o-", color=PALETTE["Baseline"], label="Manual PID")
        ax.plot(pairs, candidate, "o-", color=PALETTE["HBO pick"], label="HBO candidate")
        ax.set(xticks=list(pairs), xlabel="Pair", ylabel="Tracking score")
        ax.legend(frameon=False)
        save(fig, "16_paired_hbo_scores")

    comparison = [(name, valid_trials(group)) for name, group in trial_groups(attempts, meta).items()]
    comparison = [(name, group) for name, group in comparison if group]
    if comparison:
        fig, axs = frame("Measured controller performance", "Dots are completed trials. Bars mark each group median.",
                         1, 3, (12, max(4.8, 2.5+0.48*len(comparison))))
        labels = [f"{name}  (n={len(group)})" for name, group in comparison]
        fig.subplots_adjust(left=.23, right=.95, wspace=.35)
        for column, (field, title) in enumerate((("score", "Tracking score ↓"), ("mae", "Mean absolute error [°]"),
                                                 ("steady", "Final-window error [°]"))):
            ax = axs[0, column]
            for i, (_, group) in enumerate(comparison):
                values = series(group, field)
                ax.scatter(values, np.full(len(values), i), s=25, color=TEAL, zorder=4)
                median = np.median(values)
                ax.vlines(median, i-.23, i+.23, color=INK, lw=1.5, zorder=5)
            ax.set(yticks=range(len(comparison)), yticklabels=labels if column == 0 else [],
                   xlabel=title, ylim=(len(comparison)-.5, -.5))
        save(fig, "11_controller_comparison")

    dips = list(dip_windows(data))
    if dips:
        with (out / "disturbance_metrics.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(("source", "mode", "estimated_start_s", "target_deg", "kp", "ki", "kd",
                             "pre_dip_error_deg", "peak_abs_error_deg", "recovery_to_target_s",
                             "recovery_to_pre_dip_angle_s"))
            for window in dips:
                writer.writerow((source, window["group"], window["elapsed_s"], window["target"], *window["gains"],
                                 window["pre_error"], window["peak"], window["recovery"],
                                 window["equilibrium_recovery"]))
        for index, window in enumerate(dips, 1):
            fig, axs = frame(f"{window['group']} response to a command dip",
                             "The Uno applied −4 µs for 2 s. The lower panel shows the pulse sent to the ESC.",
                             2, figsize=(9, 5.6))
            t, rows = window["t"], window["rows"]
            for ax in axs[:, 0]:
                ax.axvspan(0, window["removal"], color=ORANGE, alpha=.13)
            ax, bx = axs[:, 0]
            target = window["target"]
            ax.axhspan(target-1, target+1, color=TEAL, alpha=.10)
            ax.axhline(target, color=MUTED, ls="--", label="Target")
            ax.plot(t, window["angle"], color=TEAL, label="Measured angle")
            ax.set(ylabel="Arm angle [°]")
            ax.legend(frameon=False, fontsize=8)
            bx.step(t, series(rows, "esc_us"), where="post", color=ORANGE)
            bx.set(xlabel="Time from applied dip [s]", ylabel="ESC pulse [µs]")
            save(fig, f"12_command_dip_{index:02d}")
        conditions = {}
        for window in dips:
            kp, _, _ = window["gains"]
            conditions.setdefault((window["target"], kp), {})[window["group"]] = window
        matched = max(conditions.values(), key=len)
        if len(matched) >= 2:
            fig, axs = frame("Disturbance comparison", "Most recent run per controller at the same target and Kp.",
                             2, figsize=(9, 5.6))
            ax, bx = axs[:, 0]
            target = next(iter(matched.values()))["target"]
            ax.axhspan(target-1, target+1, color=TEAL, alpha=.10)
            ax.axhline(target, color=MUTED, ls="--", lw=1, label="Target")
            for panel in (ax, bx):
                panel.axvspan(0, 2, color=ORANGE, alpha=.12)
            for name, color in (("P", MUTED), ("PD", ORANGE), ("PID", TEAL)):
                if name in matched:
                    window = matched[name]
                    ax.plot(window["t"], window["angle"], color=color, label=name)
                    bx.step(window["t"], series(window["rows"], "esc_us"), color=color, where="post")
            ax.legend(frameon=False, fontsize=8, ncol=4)
            ax.set(ylabel="Arm angle [°]")
            bx.set(xlabel="Time from applied dip [s]", ylabel="ESC pulse [µs]")
            save(fig, "14_disturbance_comparison")

    holds = list(matched_holds(data, events))
    if holds:
        with (out / "matched_hold_metrics.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(("source", "trial", "controller", "target_deg", "mae_12_to_18_s_deg",
                             "signed_error_12_to_18_s_deg", "ripple_12_to_18_s_deg", "peak_angle_deg"))
            for hold in holds:
                writer.writerow((source, *(hold[k] for k in ("trial", "controller", "target", "mae", "bias",
                                                             "ripple", "peak_angle"))))
        if len({hold["target"] for hold in holds}) == 1:
            fig, axs = frame("Matched angle holds", "Each line is one run. The shaded 12–18 s interval is scored before any dip.",
                             1, 2, (11, 4.6))
            ax, bx = axs[0]
            target = holds[0]["target"]
            palette = {"P": MUTED, "PD": ORANGE, "PID": TEAL}
            groups = list(dict.fromkeys(hold["controller"] for hold in holds))
            ax.axhline(target, color=MUTED, ls="--", label="Target")
            ax.axvspan(12, 18, color=TEAL, alpha=.08)
            for index, name in enumerate(groups):
                subset = [hold for hold in holds if hold["controller"] == name]
                color = palette[name]
                for j, hold in enumerate(subset):
                    ax.plot(hold["t"], hold["angle"], color=color, alpha=.75, label=name if j == 0 else None)
                errors = [hold["mae"] for hold in subset]
                bx.scatter(np.full(len(errors), index), errors, color=color, s=32, zorder=4)
                bx.hlines(np.mean(errors), index-.20, index+.20, color=INK, lw=1.7)
            ax.set(xlim=(0, 18), xlabel="Time from hold start [s]", ylabel="Angle [°]")
            ax.legend(frameon=False, fontsize=9)
            bx.set(xlim=(-.5, len(groups)-.5), xticks=range(len(groups)), xticklabels=groups,
                   ylabel="Mean absolute angle error [°]", ylim=(0, None))
            save(fig, "15_matched_hold_comparison")

    demo = None
    for event in events:
        if event["event"] == "Demo started":
            demo = (float(event["elapsed_s"]), None, "")
        elif demo and demo[1] is None and event["event"] in ("Demo complete", "Demo incomplete", "STOP", "Safety cutoff"):
            demo = (demo[0], float(event["elapsed_s"]), event["event"])
    if demo and demo[1] is not None:
        start, end, outcome = demo
        rows = [r for r in data if start <= float(r["elapsed_s"]) <= end and r["angle_deg"] and r["mode"] == "1"]
        if len(rows) > 20:
            fig, axs = frame("Timed PID demonstration", f"{outcome} · measured angle, target, and ESC command.",
                             2, figsize=(9, 5.7))
            ax, bx = axs[:, 0]
            t = series(rows, "elapsed_s")-start
            ax.plot(t, series(rows, "angle_deg"), color=TEAL, lw=1.6, label="Measured")
            ax.plot(t, series(rows, "target_deg"), color=MUTED, ls="--", label="Target")
            bx.step(t, series(rows, "esc_us"), color=ORANGE, where="post")
            offset = t[[r.get("disturbance_us") == "-4" for r in rows]]
            if offset.size:
                for panel in (ax, bx):
                    panel.axvspan(offset[0], offset[-1], color=ORANGE, alpha=.15)
            ax.set(ylabel="Arm angle [°]")
            ax.legend(frameon=False, fontsize=9)
            bx.set(xlabel="Time from demo start [s]", ylabel="ESC pulse [µs]")
            save(fig, "16_timed_demo")

    terms = [r for r in data if r["mode"] == "1" and r.get("p_us") not in (None, "")]
    if terms:
        end = int(terms[-1]["device_ms"])
        start = end - 12000
        window = [r for r in terms if start <= int(r["device_ms"]) <= end]
        if window:
            fig, axs = frame("Controller effort", "Firmware telemetry · proportional, integral and gyro derivative terms.",
                             1, figsize=(8, 4.5))
            ax = axs[0, 0]
            t = (series(window, "device_ms")-int(window[0]["device_ms"]))/1000
            for field, color, legend in (("p_us", TEAL, "P"), ("i_us", ORANGE, "I"), ("d_us", MUTED, "D")):
                ax.plot(t, series(window, field), color=color, label=legend)
            ax.set(xlabel="Window time [s]", ylabel="PID term [µs]")
            ax.legend(frameon=False)
            save(fig, "13_pid_terms")

    markers = [r for r in events if r["event"] in ("Poke", "Added load")]
    if markers:
        fig, axs = frame("Disturbance rejection", "Markers were recorded by the operator during the hold test.",
                         2, figsize=(9, 5.8))
        t0 = float(markers[0]["elapsed_s"])
        t1 = float(markers[-1]["elapsed_s"])
        rows = [r for r in data if r["experiment"] == "Hold" and r["angle_deg"]
                and t0-3 <= float(r["elapsed_s"]) <= t1+8]
        if rows:
            t = series(rows, "elapsed_s")-t0
            ax, bx = axs[:, 0]
            ax.plot(t, series(rows, "angle_deg"), color=TEAL, label="Measured")
            ax.plot(t, series(rows, "target_deg"), ls="--", color=MUTED, label="Target")
            ax.legend(frameon=False, fontsize=8)
            bx.step(t, series(rows, "esc_us"), where="post", color=ORANGE)
            ax.set(ylabel="Angle [°]")
            bx.set(ylabel="ESC [µs]", xlabel="Time from first marker [s]")
            for marker in markers:
                for a in (ax,bx):
                    a.axvline(float(marker["elapsed_s"])-t0, color=MUTED, ls=":")
                ax.text(float(marker["elapsed_s"])-t0, .87, " "+marker["event"],
                        transform=ax.get_xaxis_transform(), color=INK, fontsize=8)
            save(fig, "07_disturbance_rejection")
        else:
            plt.close(fig)

    imu = [r for r in data if r["angle_deg"] and r["gravity_ok"] == "1"]
    preferred = [r for r in imu if r["experiment"] == "Noise"] or [
        r for r in imu if r["experiment"] == "Hold"] or imu
    if len(preferred) >= 25:
        preferred = preferred[-600:]
        t = series(preferred, "elapsed_s")
        t -= t[0]
        fig, axs = frame("IMU angle estimation", "Accelerometer-only tilt and fused gyro/accelerometer tilt from the same sensor.")
        ax = axs[0, 0]
        ax.plot(t, series(preferred, "accel_angle_deg"), color=MUTED, alpha=.55, label="Accelerometer tilt")
        ax.plot(t, series(preferred, "angle_deg"), color=TEAL, linewidth=1.5, label="Complementary filter")
        ax.set(xlabel="Time [s]", ylabel="Tilt [°]")
        ax.legend(frameon=False, fontsize=9)
        save(fig, "08_imu_filter")
    active = [r for r in data if r["mode"] == "1" and r["angle_deg"] and r["error_deg"]]
    if len(active) >= 25:
        end = float(active[-1]["elapsed_s"])
        start = max(0, end-20)
        rows = [r for r in data if start <= float(r["elapsed_s"]) <= end]
        t = series(rows, "elapsed_s")-start
        error = np.array([float(r["error_deg"]) if r["mode"] == "1" and r["error_deg"] else np.nan
                          for r in rows])
        fig, axs = frame("Tracking error and ESC command",
                         "Last 20 seconds of the sampled control window. The dotted line is the ESC protection limit.",
                         2, figsize=(8, 5.5))
        ax, bx = axs[:, 0]
        ax.axhline(0, color=MUTED, linewidth=.8, linestyle="--")
        ax.plot(t, error, color=TEAL, linewidth=1.4)
        ax.set(ylabel="Target − angle [°]")
        bx.step(t, series(rows, "esc_us"), where="post", color=TEAL, label="ESC command")
        bx.step(t, series(rows, "limit_us"), where="post", color=ORANGE, ls=":", label="ESC protection limit")
        bx.set(xlabel="Window time [s]", ylabel="ESC [µs]")
        bx.legend(frameon=False, fontsize=8)
        save(fig, "09_tracking_error")

    imu_rows = [r for r in data if r["imu_ok"] == "1"]
    if len(imu_rows) >= 25:
        end = float(imu_rows[-1]["elapsed_s"])
        start = max(0, end-12)
        rows = [r for r in data if start <= float(r["elapsed_s"]) <= end]
        t = series(rows, "elapsed_s")-start
        valid = np.array([r["imu_ok"] == "1" for r in rows])
        accel = np.linalg.norm([series(rows, axis) for axis in ("ax", "ay", "az")], axis=0)/16384
        gyro = np.linalg.norm([series(rows, axis) for axis in ("gx", "gy", "gz")], axis=0)/131
        fig, axs = frame("IMU signal quality", "Raw vector magnitudes. Acceleration outside the band is rejected for tilt correction.",
                         2, figsize=(8, 5.5))
        ax, bx = axs[:, 0]
        ax.axhspan(.72, 1.28, color=TEAL, alpha=.10)
        ax.axhline(1, color=MUTED, lw=.8, ls="--")
        ax.plot(t, np.where(valid, accel, np.nan), color=TEAL)
        ax.set(ylabel="Acceleration [g]")
        bx.plot(t, np.where(valid, gyro, np.nan), color=TEAL)
        bx.set(xlabel="Window time [s]", ylabel="Gyro activity [°/s]")
        save(fig, "10_imu_signal_quality")
    return saved


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    for result in export(parser.parse_args().session):
        print(result)
