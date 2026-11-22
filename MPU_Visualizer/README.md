# MPU visualizer and flat bias calibration

A separate sketch for checking the real MPU movement on the circular screen.
It uses the same wiring and libraries as Gravity_Sand. The two sketches share
the driver and calibration math in `../shared` and the same saved offsets.
The shared I2C bus runs at 400kHz to reduce time spent fetching each sample;
both the MPU6050 and CST816D support this rate.

## What to look for

- **Cyan ring/line:** current acceleration projected into screen gravity,
  after applying saved bias corrections.
- **Gold dot/line:** the filtered gravity signal that drives the sand.
- **gx/gy:** screen-space gravity, in g. Right and down are positive.
- **Z and |g|:** corrected sensor Z and total acceleration. A stationary,
  flat, chip-side-up MPU should show Z near +1g and |g| near 1g.
- **Gyro:** corrected rotation rates for physical X/Y/Z, in degrees/sec.
- **Hz/fps:** measured fresh sample rate and scope drawing rate. The MPU is
  configured at 200Hz in FAST/RAW; the measured rate can be lower because the
  shared I2C bus and LCD still take time. Samples are acquired between LCD bands.

The scope ring is 1g. Shaking can produce larger acceleration and reach its edge;
this is expected. The scope shows projected acceleration, not an integrated
position or compass heading.

## Calibration

1. Place the **MPU module itself** flat on a level surface, **components/chip
   facing up** (+Z upward). It is the sensor's pose that matters, even if the
   LCD is held separately.
2. Tap **CAL**. There is a three-second countdown to remove your hand.
3. Keep it still until the 400 fresh samples finish, usually another 2–3 seconds.
4. `CAL SAVED` confirms offsets were written. Then tilt the MPU and check the
   pointer direction and whether the gold marker follows the cyan marker promptly.

Movement, incorrect orientation, missing readings, and implausible results are
rejected. A rejected attempt preserves the previous calibration. Tap STOP or
send `q` during the countdown/capture to cancel.

This is **flat bias calibration**: it adjusts accelerometer and gyro offsets,
preserving the expected +1g on Z. It does not determine scale errors or replace
a six-position accelerometer calibration. It also cannot distinguish physical
tilt from offset unless the module is actually level during capture.

Calibration is stored as one versioned, checksummed NVS record. Reboot and
ordinary sketch uploads preserve it. Gravity_Sand loads the same record on boot;
mount-axis corrections are also shared with the sand sketch.

## Response modes

Tap the right-hand button to cycle modes:

| Mode | MPU sample rate | Chip acceleration filter | Software smoothing |
| --- | --- | --- | --- |
| FAST | 200Hz | 94Hz, documented 3ms delay | 8ms time constant |
| RAW | 200Hz | 94Hz | None |
| OLD | 100Hz | 21Hz, documented 8.5ms delay | 40.25ms time constant |

RAW means no extra software filtering; the chip filter remains active. FAST
reduces lag while keeping the pointer quiet. OLD approximates the earlier sand
input and makes its extra delay visible. Smoothing uses actual sample times.
Mode selection is temporary; each sketch boots in FAST.

The previous fixed alpha=0.22 at 100Hz corresponds to about a 40ms time constant
and about 93ms to follow 90% of a step. These are calculated filter responses,
not measured end-to-end display latency.

Filter registers and delays are documented in the
[InvenSense register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf).

## Serial commands (115200 baud)

| Key | Action |
| --- | --- |
| `c` | Start a flat chip-up calibration, or stop an active attempt |
| `q` | Cancel active calibration |
| `f` / `u` / `o` | FAST / RAW / OLD |
| `s` | Swap physical X/Y for screen mapping |
| `x` / `y` | Reverse the corresponding screen gravity axis |
| `i` | Print sensor status and saved offsets |
| `p` | Toggle 20Hz CSV telemetry |
| `z` | Clear saved calibration explicitly |
| `?` | Print help |

Axis changes affect screen mapping and are saved for both sketches. They do
not change the physical chip-up pose required for calibration. Default mapping
is screen gravity `(-accelX,+accelY)`, matching the sand sketch.

## Build, upload, and return to sand

Run from `_CODE` in PowerShell. Keep the `shared` folder beside both sketches.

```powershell
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3:USBMode=hwcdc,CDCOnBoot=default --build-path .\build\MPU_Visualizer .\MPU_Visualizer
arduino-cli upload --port COM23 --fqbn esp32:esp32:XIAO_ESP32S3:USBMode=hwcdc,CDCOnBoot=default --input-dir .\build\MPU_Visualizer .\MPU_Visualizer
```

To return to sand with the new offsets and responsive input:

```powershell
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3:USBMode=hwcdc,CDCOnBoot=default --build-path .\build\Gravity_Sand .\Gravity_Sand
arduino-cli upload --port COM23 --fqbn esp32:esp32:XIAO_ESP32S3:USBMode=hwcdc,CDCOnBoot=default --input-dir .\build\Gravity_Sand .\Gravity_Sand
```
