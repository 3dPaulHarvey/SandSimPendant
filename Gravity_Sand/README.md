# Gravity Sand

1,200 colored 3-pixel grains fall, collide, and pile inside the 240×240 circular
screen. Tilt the MPU6050 to change gravity. This uses the accelerometer; a gyro
orientation filter is unnecessary for this first pass. A face-up screen has very
little gravity in its plane, so grains stop drifting after their momentum fades.

For responsiveness and calibration, use the separate
[`MPU_Visualizer`](../MPU_Visualizer/README.md) sketch. Both sketches load the
same saved flat-bias calibration. Sand now uses 200Hz MPU samples, a 94Hz chip
filter, and time-based 8ms software smoothing, replacing the old 100Hz/21Hz
filter and fixed alpha=0.22. Sensor reads also run between LCD strips.

The original `../Circlular_Touchscreen` sketch is preserved.

## Tuning and uploading

Edit the documented values at the top of [`SandConfig.h`](SandConfig.h). This is
the single place for sand feel, grain count, update rates, display batching,
shake strength, and button settings. Positions and speeds use 3-pixel cells.

| Setting | Default | Effect of increasing it |
| --- | --- | --- |
| `kSlideRate` | 70 | Grains roll off piles sooner |
| `kMinimumSlideRate` | 12 | Shallow tilts release piles sooner |
| `kRollingRetention` | 0.75 | Grains keep more momentum after rolling |
| `kAcceleration` | 240 | Grains start moving faster |
| `kMaxSpeed` | 90 | Higher falling speed, with more automatic safety substeps |
| `kTurnAccelerationBoost` | 1.4 | Stronger brief response to a direction change |
| `kTurnBoostSeconds` | 0.12 | Longer direction-change boost |
| `kParticleCount` | 1200 | More sand, more simulation and drawing work |
| `kRenderMergeGapCells` | 4 | Fewer LCD commands, more unchanged pixels resent |
| `kRenderBandRows` | 4 | Taller drawing batches; range 1–8 |
| `kRenderMergeOverdrawCells` | 8 | More vertical merging, at the cost of extra pixels |
| `kShakeGain` | 1.75 | More agitation; `+`/`-` also adjusts this live |

Direction changes of at least 45 degrees release rolling immediately, reduce
old uphill momentum, and briefly boost acceleration. Small input fluctuations
do not repeatedly trigger this response. The circular-wall slide treatment is
retained. Keep the 60Hz physics/display targets for responsiveness. Internal
substeps automatically keep each move below one cell if speed settings change.

After saving a variable change, run this from the `_CODE` folder in PowerShell:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\upload-sand.ps1 -Port COM23
```

This recompiles before uploading, so header changes are included. Replace
`COM23` with the connected board's current port. Close Serial Monitor before
uploading. To compile without a board, append `-BuildOnly`. The helper finds
Arduino CLI on PATH or in the installed Arduino IDE and keeps intermediate
build files outside Dropbox to avoid sync-related file locks.

## Shake and jolts

The existing X/Y acceleration signal already includes the effect of moving the
container. The shake response now also uses all three accelerometer axes to
loosen and agitate the grains, including bumps along the MPU's normal/Z axis.
Tilt continues to drive the same downhill force.

A gyro-guided gravity estimate separates ordinary rotation from dynamic
acceleration for the agitation detector. Brief shocks are accumulated between
physics updates so a short bump is not lost between display frames. The
additional random impulses are bounded and balanced. Transient X/Y acceleration
also gets an extra directional boost so a shake makes a visible tabletop shove;
this is a deliberate amplification for feel, applied only to detected movement.
Ordinary tilt retains its original force. In this 2D simulation, normal/Z bumps are represented by
in-plane grain agitation rather than simulated height.

Shake response is enabled in automatic MPU mode (`0`) by default. Try a gentle
side-to-side shake and a short up/down bump of the MPU. Serial `h` toggles the
added shake response for comparison, `+`/`-` adjusts its strength (0.25–3.0, default
1.75), and `t` applies a test jolt. Shake settings reset on reboot. Quiet sensor
noise stays below the detector's deadzone, and disconnect/reconnect resets its
history. Saved calibration is preserved.

The sensor measures acceleration from gravity and motion together; this is why
the physical X/Y force already reacted to some shaking before the added
agitation. See [Analog Devices AN-1057](https://www.analog.com/en/resources/app-notes/an-1057.html).

## Refresh and performance

The current build targets **60 display updates per second**, with the same
1,200 grains. LCD SPI starts at **40 MHz**, up from the original 8 MHz. Only
changed rectangles are transferred; clearing/redrawing the whole screen would
limit a 40 MHz RGB565 connection to about 43 complete frames/sec before
command overhead. Scheduled frame deadlines prevent polling jitter from
accumulating into a lower update rate.

Earlier single-task measurements on COM23 with repeated right/up/left/down
gravity changes: the original build ran about **17–24 FPS** during movement;
the optimized single-task version ran **58–60 FPS**,
including the scatter/reset phase, with 1,200 grains and no I2C read errors.
Typical physics work dropped from roughly 8–12 ms to 3–4 ms per update. Short
individual frames can still exceed the target during a full scatter; the FPS
figures are two-second averages, not a guarantee for every individual frame.

The current dual-core build runs input, I2C, physics, settings, and Serial on
Arduino's loop core (core 1 with the upload helper). A dedicated LCD task runs
on the opposite core (core 0). Wi-Fi and Bluetooth are not initialized. The
renderer receives copied color-grid snapshots through a one-frame mailbox;
new snapshots replace stale pending ones rather than forming a frame backlog.
No live particle arrays are shared with the drawing task. One owner per bus
keeps LCD/SPI separate from touch/MPU I2C, and LCD work never holds a physics lock.
If LCD task allocation fails, a logged single-task fallback remains available.

Physics still defaults to two internal substeps at 60 updates/sec; safety
substeps increase automatically when a higher speed cap requires them. Shared
gravity calculations and downhill neighbor choices are prepared once per tick.
The updated motion rules intentionally favor quick release over precise physics.

Native behavioral comparisons against the frozen earlier engine showed about
26% more early pile travel after a quarter-turn: measured over the first 100ms
at full gravity and 200ms at shallow tilt. These are simulated movement tests,
not hardware input-to-screen latency measurements. On identical simulated
quarter-turn frames, the new drawing planner reduced LCD windows from about
205 to 89 per frame, while increasing pixels sent from about 6,882 to 10,793.
Actual FPS and latency for the dual-core firmware still require hardware checks.

USB `PERF` lines report completed drawing FPS, the rate of frames that actually
change pixels, drawing and physics timings, pixels sent, and SPI windows per frame.
`DISPLAY` lines also report time from snapshot capture to drawing completion
(not total sensor-to-panel latency). Use moving-sand measurements to assess
performance; a settled pile requires very little display traffic. Serial `v` switches between 40 MHz and
the original 8 MHz; reboot restores 40 MHz.

The software update rate is separate from the LCD panel's own scan rate.
The precise maximum panel refresh rate is not established by the available
[GC9A01A preliminary datasheet](https://cdn-learn.adafruit.com/assets/assets/000/135/078/original/GC9A01A.pdf).

## Hardware

Seeed XIAO ESP32-S3, GC9A01 display, CST816D touch, and MPU6050.

| MPU6050 | XIAO |
| --- | --- |
| VCC | 3V3 |
| GND | GND |
| SDA | D4 / GPIO5 — shared with touch |
| SCL | D5 / GPIO6 — shared with touch |
| INT | Unconnected |
| AD0 | Low/default for address 0x68; high also supported at 0x69 |

The program checks both MPU addresses every two seconds. Without an MPU, sand
falls toward screen-bottom. Connecting or reconnecting the MPU switches back to
live tilt automatically, unless a manual/demo mode is selected.

**D8/D10 and the LCD's pins labeled SDA/SCL are SPI display signals.** Keep the
display there, and connect the MPU to the touch I2C bus on D4/D5 instead. Use the
MPU's SDA/SCL pins, not its auxiliary XDA/XCL pins. If the MPU is missing, the
display shows `NO MPU6050` and the required D4/D5 wiring above the sand.

Existing display wiring: CS=GPIO1, DC=GPIO2, RST=GPIO4, SCK=GPIO7, MOSI=GPIO9;
display VIN/BLK=3V3. Touch reset=GPIO43, interrupt=GPIO44. Touch and MPU share
GPIO5/6 at 400 kHz, supported by both the MPU6050 and CST816D.

## Two digital buttons

The remaining unused D0–D10 header pins are **D2/GPIO3** and **D9/GPIO8**,
confirmed against the installed XIAO ESP32-S3 board variant and
[Seeed's pin table](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/).

| Header pins | Current use |
| --- | --- |
| D0/GPIO1, D1/GPIO2, D3/GPIO4 | LCD CS, DC, reset |
| D4/GPIO5, D5/GPIO6 | Shared touch/MPU I2C |
| D6/GPIO43, D7/GPIO44 | Touch reset, interrupt |
| D8/GPIO7, D10/GPIO9 | LCD clock, MOSI |
| **D2/GPIO3** | **Scatter button** |
| **D9/GPIO8** | **Palette button** |

Use two **normally open momentary switches**. Connect one switch between D2
and GND, and the other between D9 and GND. The code enables `INPUT_PULLUP`, so
pressing reads LOW; no connection to 5V is needed. Unconnected switches leave
the inputs high. Each press is debounced for 25ms and triggers once. Holding a
button at startup does nothing until it is released and pressed again.

D9 is normally the SPI MISO pin, but this LCD is write-only and the sketch
explicitly disables MISO with `SPI.begin(LCD_SCK, -1, LCD_MOSI, LCD_CS)`.
Do not attach an SPI device that needs MISO while using the D9 button.

GPIO3 also selects the JTAG source at reset **if the `STRAP_JTAG_SEL` eFuse was
enabled**; with default eFuses its level does not alter that selection. This
does not select the normal boot/download mode. An optional 10k pull-up from D2
to 3V3 establishes a high reset level; if your board uses strap-selected JTAG,
release D2 during reset to retain USB JTAG. No eFuse change is required.
See [Espressif's JTAG source explanation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/jtag-debugging/configure-other-jtag.html).

Button GPIOs and debounce time are editable in `SandConfig.h`. Compile-time
checks reject collisions with the existing LCD, touch, I2C, and USB pins.

The MPU's starting orientation assumes its +X arrow points screen-right and its
+Y arrow points screen-up, using screen gravity `(-accelX, +accelY)`. Mounting
corrections can be changed over USB without recompiling.

## Controls

Tap and release the screen to scatter the grains again. The small dot at the
top is green with an MPU, amber while waiting, and cyan in manual/demo mode.

Serial monitor at 115200 baud; commands are single characters:

| Command | Action |
| --- | --- |
| `r` | Scatter/reset sand |
| `p` | Cycle warm, blue, and purple palettes |
| `0` | Automatic MPU tilt, or downward fallback if disconnected |
| `1`, `2`, `3`, `4` | Force gravity down, left, up, right |
| `d` | Toggle slowly rotating demo gravity |
| `s` | Swap the two MPU axes |
| `x`, `y` | Reverse gravity on the corresponding screen axis |
| `v` | Toggle LCD SPI between the original 8 MHz and faster 40 MHz |
| `i` | List responding I2C addresses |
| `m` | Check MPU wiring, address ACKs, identity, and raw acceleration |
| `h` | Toggle the added XYZ shake agitation |
| `+` / `-` | Increase / decrease shake strength |
| `t` | Apply a test shake impulse |
| D2 button | Scatter/reset sand |
| D9 button | Cycle palettes |
| `?` | Print help |

Axis corrections survive reboot. Send the same correction command again to
undo it. Screen rotation is fixed at the original setting, zero.

## Build and upload

Uses the already installed Adafruit GC9A01A, Adafruit GFX, and Adafruit BusIO
libraries. MPU registers are read directly with checked I2C transactions.
Board package: Espressif ESP32 3.3.12. PSRAM is not required.

From the `_CODE` directory in PowerShell:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\upload-sand.ps1 -Port COM23
```

Arduino IDE equivalents: XIAO_ESP32S3; USB Mode=Hardware CDC and JTAG;
USB CDC On Boot=Enabled; PSRAM=Disabled; Arduino Runs On=Core 1.

To restore the previous touch demo, compile/upload the unchanged
`Circlular_Touchscreen` folder with the same board and port.

## Code and checks

`Gravity_Sand.ino` handles hardware, gravity mapping, buttons, touch, and the
two-core frame pipeline. `DirtyRenderer.h` plans bounded drawing rectangles;
`ButtonInput.h` debounces physical inputs. `SandSimulation.h` is an
allocation-free, portable particle/grid simulation. Edit `SandConfig.h` to
adjust sand quantity and behavior; the
current grid and renderer use 3×3-pixel grains.

Keep `../shared/MotionCalibration.h` and `../shared/MPU6050Motion.h` beside the
sketch folders. They provide the common sensor driver, calibration, and filtering.

`../tests/sand_sim_test.cpp` checks containment, unique occupancy, grain count,
directional settling, zero gravity, deterministic reset, bad inputs, and 100
simulated seconds of rotating gravity. With a native C++ compiler:

```powershell
g++ -std=c++11 -O2 -Wall -Wextra -Werror .\tests\sand_sim_test.cpp -o .\tests\sand_sim_test.exe
.\tests\sand_sim_test.exe
```

Shake checks are in `shake_input_test.cpp` (sensor motion extraction, timing,
quiet rest, pure tilt, and still-pose recovery), `shake_sim_test.cpp` (bounded
grain impulses and containment), and `shake_integration_test.cpp` (the complete
sensor-to-grain path at 180/200Hz input and 60Hz physics). The integration check
compares real particle travel with and without the amplified shake response.

`sand_response_test.cpp` compares early full/shallow turn response with the
frozen original engine. `button_input_test.cpp` covers bounce, holding,
boot-held switches, independent buttons, configurable debounce, and clock
wraparound. `dirty_renderer_test.cpp` verifies exact repaint results across
random, checkerboard, palette/status invalidation, and simulated-motion frames;
it also prints LCD-window/pixel comparisons. Build these the same way as the
native example above. The old `artifacts/compare_engine.cpp` exact-state
comparison predates the intentional behavior changes and is not an acceptance
test for this version.
