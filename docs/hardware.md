# Hardware

## Parts

| Part | Reference build |
| --- | --- |
| Controller | Arduino Uno, ATmega328P, USB cable |
| Angle sensor | MPU6050 on an I²C breakout, rigidly attached to the arm |
| Motor | A2212/13T brushless outrunner, 1000 KV, with a compatible propeller |
| Motor drive | SimonK 30 A ESC with pulse-width input |
| Motor power | Separate 12 V DC supply compatible with the ESC and motor, rated for the actual load, with an accessible disconnect |
| Mechanics | Wooden lever, single hinge, stiff upright, weighted or fastened base, two padded mechanical stops and strain relief |

## Mechanical assembly

Fasten the upright to the base and mount a single horizontal hinge. Secure the motor at the long end so its thrust lifts the arm. Keep the propeller clear of the base, upright, stops and wiring throughout the full travel. Fasten the IMU to the moving lever. Route cables so they bend freely at the hinge without pulling the arm. Confirm the arm returns under gravity with motor power off. Fit physical stops in both directions.

On the reference rig the measured stops were −33.6° and +26° from horizontal. The software ceiling was +15.2°. Measure these angles on your own build. The firmware cuts drive at its guard, and mechanical travel continues while the arm coasts. Use a stable base, a safe propeller exclusion area and a motor-power disconnect within reach before any powered run.

## Wiring

| From | To |
| --- | --- |
| Uno USB | Laptop, for Uno power and serial data |
| Uno A4 and A5 | MPU6050 breakout SDA and SCL, respectively |
| Uno GND | MPU6050 GND and ESC signal ground |
| Uno D6 | ESC pulse signal |
| MPU6050 AD0 | GND for address 0x68 |
| Breakout VCC | Supply voltage supported by your specific breakout |
| ESC thick DC input leads | Separate motor supply with correct polarity |
| ESC three phase outputs | Three motor leads |

Use a breakout with suitable supply and I²C pull-up levels. Check the regulator and pull-up arrangement on your board variant before wiring. With the Uno powered over USB, insulate the ESC's BEC power lead if present. Keep motor current in the motor power wiring, with a shared signal ground for the pulse command. Identify the ESC leads by its markings or manual.

With the propeller removed, check motor direction and disarm before moving any wires. Swap two motor phase leads with power off if direction is reversed. Keep ESC power disconnected during IMU calibration. See [setup](setup.md) for the first closed-loop hold.
