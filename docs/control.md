# Controller and optimiser

## Real-time controller

The Uno reads the MPU6050 at nominal 100 Hz. Three calibrated poses define the plane of rotation and gyro axis. The complementary filter integrates gyro rate and corrects it toward the accelerometer estimate when the gravity check passes.

The PID loop runs at nominal 200 Hz. The ESC command is emitted at 50 Hz, with telemetry sent at 50 Hz. For angle error `e = target − measured`, the proportional term is `Kp × e`, the integral accumulates `Ki × e × dt`, and the derivative is `−Kd × gyro_rate`. The nominal pulse is the trim plus these corrections. Gains are in µs/°, µs/(°·s) and µs·s/°, respectively. The integral contribution is bounded to ±8 µs and conditional integration limits windup during saturation.

Launch advances a nearby internal reference and adds bounded boost if the arm stalls. A shared upward-speed brake reduces drive when the arm approaches the target quickly. It acts separately from the logged PID terms. The hold gains take over close to the target and at low speed. The firmware limits pulse width and upward pulse rate, disarms on stale telemetry input or invalid IMU data, and enforces angle, rate, launch and drive-time limits. The dashboard stops a hold on sustained overshoot. A software cutoff removes drive while the mechanical arm can continue to coast.

## Hardware tuning

A scored trial starts with a motor-off rest and an initial hold. Two six-second setpoint steps are scored. The objective is mean absolute angle error plus weighted overshoot and undershoot, final-window error, ripple and absolute changes between adjacent ESC commands. Lower is better. Unsafe or incomplete runs are logged, and an upper-angle abort during a scoring step receives the configured penalty.

The optimiser fits a Matérn Gaussian process to log scores in log-gain coordinates. A second Gaussian process models the scale of leave-one-out prediction errors. Augmented Expected Improvement discounts candidates with high estimated residual variability. The search starts with eight random trials and follows with 22 guided trials. The dashboard retests shortlisted settings and runs a separate matched comparison before recommending a gain change.

## Source and serial protocol

| File | Responsibility |
| --- | --- |
| [arm_controller.ino](../hardware/arm_controller/arm_controller.ino) | IMU fusion, PID, ESC pulses, command parsing and firmware cutoffs |
| [dashboard.py](../hardware/dashboard.py) | Desktop controls, live plots, calibration and results |
| [rig_link.py](../hardware/rig_link.py) | Serial I/O, telemetry decoding and calibration geometry |
| [experiments.py](../hardware/experiments.py) | Test sequencing, logging and safety supervision |
| [hbo.py](../hardware/hbo.py) | Scoring, Gaussian processes and gain selection |
| [rig_sim.py](../hardware/rig_sim.py) | Illustrative plant for practice mode |
| [make_plots.py](../hardware/make_plots.py) | Offline export from saved logs |

The USB serial link uses 115200 baud. Commands are newline-terminated. `H` is heartbeat, `A` arms, `X` disarms, `S` stops drive, `G` sets idle-mode gains, `J` hands over gains during a settled hold, `T` sets the target, `R` starts PID, `P` requests a fixed pulse, `D` requests a timed dip, `K` sends pose calibration, and `L` sets trim and pulse cap while disarmed. The Uno emits `ARM3` telemetry and command acknowledgements. Use the dashboard for powered operation so the host supervisor remains active.
