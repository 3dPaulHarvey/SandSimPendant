# SandSimPendant

A tilt-responsive fluid pendant using an **Aideepen ESP32-S3 Super Mini**, a
standalone **GC9A01 1.28-inch 240 x 240 round display**, a **GY-521 MPU6050**,
and two optional tactile buttons. The latest verified board uses **COM32**; upload
scripts automatically select the connected Espressif serial port when it is unique.

The pre-effects baseline is saved as **`pre-effects-2026-11-27`**. See the
[checkpoint notes](CHECKPOINTS.md) for validation results and recovery steps.

## Start with the display and IMU test

Wire the modules using the [hardware catalog](HARDWARE.md#project-wiring), then:

```powershell
.\upload-imu.ps1 -BuildOnly
.\upload-imu.ps1
```

The [IMU visualizer](MPU_Visualizer/README.md) starts with an RGB display check,
then shows a moving tilt indicator, accelerometer and gyro readings, and sensor
status. It runs without a serial monitor or buttons attached. See its README
for calibration and controls.

To run the fluid test:

```powershell
.\upload-sand.ps1
```

The [fluid test](Gravity_Sand/README.md) replaces the old grid-based sand with
particles moving continuously in pixel space. It starts with circles covering
about 22% of the screen. GPIO10 triggers a random explosion at 100%-200% of the
previous maximum strength (600-1,200 px/s); the **randomize
button** on GPIO11 aims for **15-35% visible fill**, favoring the lower end.
The target follows `16 + 18*u^2` percent, for a mean of about **22%**. Crowded
bonus rolls have been removed. Coverage counts the visible union after overlap
and circular clipping, then approximately fits size/count once on randomize.
Sizes range from **100-350% of the original 10px baseline** (10-35px
before motion stretch). Preferred sizes are drawn uniformly and fitted to the chosen fill,
with at least five dots to retain every foreground color. Physical radii and
spacing scale with the visuals, which remain twice the physical diameter for
overlap. This calculation adds no work to normal animation frames.
That button also chooses a different visual shape (circle, triangle, square,
or five-point star), one of 21 locked color swatches, and a new moving
gradient effect. Each swatch provides five dot colors and three dark background
hues. Three overlapping soft shapes wander at about 8–18px/s while their
brightness and saturation pulse independently. The Great Wave is the startup
default; serial `p` cycles swatches with a new background, and `g` rerolls only
the background effect. All foreground shapes keep the same round-particle
physics and velocity-based squash/stretch. Each randomization uniformly chooses
a stretch cap from 1x to 3x for the whole population; reciprocal squash preserves
area. Wall impacts briefly squash against the rim and rebound without changing
the circular physics. On boot and every Randomize press, exactly one particle uses a
different shape; all others share the chosen main shape. The main screen has
no IMU indicators, error flashes, or connection text.
Error counts and detailed diagnostics remain available over serial.
The onboard RGB LED also follows tilt as a simple color wheel: up is blue,
right green, down yellow, and left red, blending between directions. It fades
off near level and brightens with tilt. Both main sketches reuse the existing
filtered sensor readings, with at most 25 normal LED updates per second and
no trigonometry, square roots, or extra sensor polling for the LED.
Background redraws use an adaptive
tile budget and skip areas hidden by opaque foreground shapes, keeping motion
active under load while reserving drawing time for the particles.
Browse the [21 swatches](Gravity_Sand/assets/color-swatches.svg) or their
[supplied color data](Gravity_Sand/assets/color-swatches.json).
See the [fill probability plot](Gravity_Sand/assets/fluid-size-count.png) and
[button wiring and controls](Gravity_Sand/README.md#controls).
It targets 240 physics updates
and 60 drawing snapshots per second, with the LCD on the second core. These
are scheduling targets; serial telemetry reports the achieved rates.
The earlier 2026-11-22 baseline runs sustained about 60 completed software
frames per second; see the [hardware measurements and current input-health
behavior](Gravity_Sand/README.md#scheduling-and-diagnostics).

## Shared setup

All sketches use raw GPIO numbers from [`shared/BoardConfig.h`](shared/BoardConfig.h).
The reusable driver, calibration, and button debounce code live in `shared`.
Keep that folder beside the sketch folders.

`upload-firmware.ps1` owns the common build/upload settings: **ESP32S3 Dev Module**,
240 MHz, 4 MB flash, DIO, default 4 MB partitions, PSRAM disabled, native USB CDC
enabled, and loop on core 1. It compiles before uploading and uses a watchdog
reset to restart the board. Build intermediates live in
`%LOCALAPPDATA%\SimCircle\build`; logs live in the ignored repository `build` folder.
Close Serial Monitor before uploading. If multiple Espressif boards are connected,
select one explicitly, for example `-Port COM32`. An explicit port is never
silently redirected to a different board. `-BuildOnly` needs no connected board.

Installed dependencies: Arduino CLI, Espressif ESP32 core 3.3.12, Adafruit
GC9A01A, Adafruit GFX, and Adafruit BusIO. No extra IMU library is required.

## Project contents

- [Hardware catalog](HARDWARE.md): current parts, specifications, and wiring.
- [MPU_Visualizer](MPU_Visualizer/README.md): screen/IMU diagnostic and calibration.
- [Gravity_Sand](Gravity_Sand/README.md): variable-size fluid firmware, tuning, and controls.
- [ESP32S3_Diagnostics](ESP32S3_Diagnostics/README.md): optional onboard RGB/USB test.
- `shared`: portable board configuration, button input, motion, and calibration.
- `tests`: native C++ checks for simulation, rendering, buttons, and motion.

Run the native C++11 checks with `.\tests\run-native-tests.ps1`; see the
[test instructions](Gravity_Sand/README.md#code-and-checks).
