# Experiments

## Measured PID result

The reference arm ran three PID holds and two PD holds at −9° with the same 1140 µs trim, proportional gain and derivative gain. The run-level means use the same 12–18 s window. PID enabled the integral term at 0.13 µs/(°·s).

| Outcome | PD | PID |
| --- | ---: | ---: |
| Mean absolute hold error | 2.50° | 1.40° |
| Recovery to within ±1° for at least 1 s following the dip | 1 / 2 | 3 / 3 |

The disturbance was a −4 µs ESC-command offset applied for two seconds. This tests rejection of a reproducible actuator-command change. The [matched holds](examples/pd-pid-holds/) include telemetry, event labels and session settings.

[Measured angle traces and hold errors](images/hold-comparison.png)

## Bayesian tuning result

The completed [search dataset](examples/hbo-search/) contains the scored random, guided, baseline and shortlist runs. The best guided trial scored **6.10**, compared with **6.48** for the best random trial. The chosen candidate averaged **11.21** over its three retests and the manual baseline averaged **5.97**. The manual gains were retained for the reference demonstration. Compare candidates under matched trim, targets and start conditions before adopting them on another build.

[Tracking score across the search](images/tuning-progress.png)

## Repeat the tests

1. Confirm calibration, trim and a steady hold before starting a trial.
2. In **PID bench**, compare the PD and PID presets at the same target and trim. During a steady hold, **Apply −4 µs dip** requests the disturbance. Keep every repeat and use the same scoring window.
3. In **Bayesian tuning**, choose three targets inside the calibrated range. Run the manual baseline, then the search and independent repeats using the same profile. Keep the raw logs and any aborts together with the calibration and settings.
4. In **Data**, save plots after disarming. Session files are stored under `hardware/runs/hardware/` and `hardware/runs/simulation/`. Practice-mode results are marked as simulation.

To regenerate plots from the included matched-hold telemetry, run this from the repository root after installing the dependencies:

```powershell
.\hardware\.venv\Scripts\python.exe hardware/make_plots.py docs/examples/pd-pid-holds
```

The search example contains trial-level data and does not include the full continuous telemetry stream. For a focused matched comparison against a saved guided candidate, the dashboard accepts `--matched-hbo path/to/session.json` with the associated `trials.csv`. Use a candidate calibrated for the current rig.
