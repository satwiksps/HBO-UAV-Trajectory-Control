# Setup and calibration

1. Install the Python dependencies from [requirements.txt](../hardware/requirements.txt) using the [README commands](../README.md#try-it). Run `dashboard.py --sim` first to check the interface.
2. Disconnect ESC power. In Arduino IDE, install **Arduino AVR Boards**, open [arm_controller.ino](../hardware/arm_controller/arm_controller.ino), select **Arduino Uno** and its serial port, and upload. The Uno starts disarmed and emits a 950 µs idle pulse on D6.
3. Close Serial Monitor and any motor bench utility. From `hardware/`, launch `dashboard.py` or double-click **Open Propeller Arm.cmd**. Select the Uno port and click **Connect**. Verify that the IMU address and sensor data appear.
4. In **Calibration**, enter the measured lower and upper stop angles and a separate operating ceiling below the upper stop. With the ESC still unpowered, capture **Lower**, **Level**, and **Upper**, holding the arm still at each pose. Apply the calibration. Check zero at level, a negative angle at the lower stop and a positive angle at the upper stop. Confirm the calibration in the dashboard.
5. In **Sensors**, run the motor-off noise test. Check that the angle is steady with the rig at rest and increases when the motor end is raised.
6. Verify propeller clearance, secure the rig and restore ESC power. In **PID bench**, start with a conservative target already verified inside the usable travel. The reference rig used −9° and gains `0.55, 0.13, 0.25`. Click **Arm rig**, then **Hold angle**. Test **STOP** and keep the physical motor-power disconnect accessible.
7. Use the **Sensors** ramp and the logged pulse during a steady hold to choose the trim for your rig. The first level crossing is a useful starting observation. Payload, supply and hinge friction can shift the required pulse. The reference unit held −9° on a short verified run at a trim of 1135 µs.

The dashboard saves calibration locally as `hardware/calibration.json`. Record fresh poses after moving the sensor or changing the mechanics. During a successful hold, **Apply target** changes the setpoint by up to 2°/s. Use small target changes for the first test.

| Symptom | Check |
| --- | --- |
| Port busy or no telemetry | Close other serial apps. Check the USB data cable, selected port and uploaded sketch. |
| IMU absent or angle reversed | Check A4/A5, ground, power, AD0 and the three captured poses. |
| Motor beeps at idle | Check common signal ground, D6 and ESC pulse compatibility with the propeller removed. |
| Launch timeout | Review whether the arm climbed and what pulse it reached. Check hinge freedom, power delivery and trim before retesting. |
| Overshoot or cutoff | Review the event and angle/pulse traces. Verify the mechanics and calibration, then repeat a conservative hold. Keep the cutoffs active. |

## Optional bench diagnostics

[Uno health](../hardware/tools/board_health/uno_health/uno_health.ino) and [IMU bench](../hardware/tools/imu_bench/imu_bench.ino) are standalone sketches for component checks. The [ESC bench](../hardware/tools/esc_bench/esc_bench.ino) and its [GUI](../hardware/tools/esc_bench/esc_gui.py) check the ESC separately. Remove the propeller for motor-only diagnostics, verify the sketch's selected signal pin before connecting, and restore the main controller sketch before using the dashboard. The diagnostic scripts accept a serial port argument.
