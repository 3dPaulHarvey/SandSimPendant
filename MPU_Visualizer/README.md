# GY-521 visual diagnostic

A standalone screen and motion test for the Aideepen ESP32-S3 Super Mini,
GC9A01 240 x 240 round display, and GY-521 MPU6050 module. It runs immediately
without a Serial Monitor, physical buttons, or saved calibration.

## Wiring

All pin numbers are raw GPIO numbers printed on the Super Mini. The shared
[`BoardConfig.h`](../shared/BoardConfig.h) is the source of truth for both this
sketch and Gravity_Sand.

| Module pin | Super Mini |
| --- | --- |
| Display RST | GPIO4 |
| Display CS | GPIO1 |
| Display DC | GPIO2 |
| Display SDA (SPI MOSI) | GPIO9 |
| Display SCL (SPI clock) | GPIO7 |
| Display VCC / GND | 3V3 / GND |
| GY-521 SDA / SCL | GPIO12 / GPIO13 |
| GY-521 VCC / GND | 3V3 / GND |
| Optional B1 | GPIO10 to GND when pressed |
| Optional B2 | GPIO11 to GND when pressed |

GY-521 INT, XDA, and XCL are unused. AD0 low/default selects 0x68; high selects
0x69. Both addresses are detected automatically. Display SDA/SCL are **SPI**;
they do not share the IMU's **I2C** pins. No separate backlight pin is exposed
on this seven-pin display. Disconnect USB before changing wires.

Buttons use internal pull-ups and 25 ms debounce. A held button never repeats;
a button held at startup must be released before it can trigger an action.

## What the screen shows

1. A red, green, then blue full-screen sequence, 300 ms each, checks the panel's
   basic color output at startup.
2. **IMU TEST** appears with LIVE/WAITING/OFFLINE status, response mode, detected
   sensor address, and saved calibration status.
3. A cyan vector and outline circle follow in-plane acceleration (median-qualified
   in FAST mode, immediate in RAW mode);
   a gold bubble follows the filtered signal used by the sand simulation.
   The outer ring represents 1g. Stronger motion is clipped at the ring.
4. **A** lists physical X/Y/Z acceleration in g. **G** lists X/Y/Z angular rates
   in degrees per second. Sample rate, drawing FPS, and cumulative I2C errors
   appear below. A stale or missing sensor displays dashes instead of old values.

Place the GY-521 itself flat, components facing up: acceleration should be
roughly (0, 0, +1)g, magnitude near 1g, and gyro near zero while still. Tilt it
to move the bubble; rotate it to change the gyro readings. The current COM35 mount
uses screen gravity **(+accelY, -accelX)**: swap=1, flipX=1, flipY=1. These
defaults live in `shared/BoardConfig.h`; saved adjustments take precedence.
This assumes the face-flipped sensor still has its connector end at the top.
The mounted sensor reads negative Z with the screen facing up; this is separate
from the components-up pose required for bias calibration. Physical tilt
directions still need checking once the display is wired.
The four-direction check should be left->left, toward you->down, right->right,
and away from you->up. The physical acceleration/gyro number labels stay in chip
axes; the cyan vector and gold bubble use the corrected screen axes.

The visualizer targets 30 redraws/sec and uses a conservative 20 MHz SPI clock.
FAST and RAW both configure the MPU for 200 Hz sampling and the existing 94 Hz
chip filter. FAST qualifies motion with a three-sample median, then adds an
8 ms software smoothing time constant. The median rejects one isolated spike
and adds one fresh-sample interval of latency to a sustained change. After a
filter reset or reconnection, the first two samples build history; the filtered
bubble becomes available with the third. RAW keeps immediate gravity output
and bypasses this extra median/smoothing path. Reported rates are measured, not
guarantees.

The shared driver preserves raw and unfiltered calibrated measurements for the
physical A/G numbers and calibration collector. It rejects whole zero/FF sensor
packets, complete all-FF acceleration/gyro vectors, clipped axes, reserved
interrupt-status bits, and I2C read underflow without counting them as fresh
samples. Zero acceleration alone is still allowed during free
fall. The median cannot validate every plausible packet on an unreliable bus.
The visualizer still starts at the shared **400 kHz I2C** setting; Gravity_Sand
has its own **100 kHz** starting setting and a `b` clock toggle.

The installed replacement GY-521 reports nonstandard identity **0x75**. Its exact
chip model is unconfirmed. A board-specific compatibility setting permits only
that additional identity, still requiring successful configuration readback and
fresh samples. The screen shows **ID75 COMPAT**, and serial includes `id` and
`compat`. Set `hardware::kImuCompatibilityId` to zero to require standard MPU6050
identity. Other unknown IDs are rejected. Basic changing measurements were
observed at about 111 Hz with zero bus errors; filter timings and full scale
accuracy of this unidentified part are not independently verified.

If the sensor is missing, the screen shows the GPIO12/13 wiring and retries about
every two seconds. LIVE requires a fresh sample within 150 ms. The driver
reconnects after a 250 ms stale-data timeout. The screen can therefore be tested
before the GY-521 is connected. A responding sensor rejected by the identity or
configuration checks is shown as **IMU REJECTED** with its ID.

## Controls and calibration

- **B1 / serial c:** begin a calibration, or cancel one in progress.
- **B2 / serial m:** toggle FAST and RAW response.
- **Serial q:** cancel calibration.
- **Serial f / u:** select FAST / RAW directly.
- **Serial s / x / y:** swap axes / reverse screen X / reverse screen Y.
- **Serial z:** explicitly clear saved calibration.
- **Serial i / ?:** print diagnostic status / help.
- **Serial b:** scan I2C addresses at 100 and 400 kHz, report idle line levels,
  and read identity/power registers at 0x68/0x69 for responding devices. It
  restores the prior clock and reconnects the driver afterward. Drawing pauses
  briefly during this on-demand test.
- **Serial 1 / 4:** use 100 / 400 kHz I2C until reboot, then retry the IMU.
- **Serial d:** read configuration and ten raw sensor register blocks without
  bypassing the sensor identity check or changing its configuration.

Serial speed is 115200 baud. Normal operation prints one `IMU_DIAG` line each
second, including uptime, sensor status/address, sample rate, display-loop FPS,
fresh sample count/age, physical acceleration, gyro, and errors. Frame counts
confirm that drawing commands were sent; only looking at the panel confirms
physical display output.

To calibrate, put the **GY-521 board flat with its components facing upward**,
then press B1 or send `c`. After a three-second countdown, keep it still for 400
fresh readings (usually another 2-3 seconds). Movement, invalid orientation,
missing samples, and failed saves preserve the previous calibration. A success
message confirms the offsets were stored. This is flat bias calibration, not a
six-position scale calibration.

Offsets and mounting-axis corrections are shared with Gravity_Sand in NVS and
survive ordinary uploads and resets. Screen mapping does not change the physical
chip-up pose required for calibration. Changing to another board does not copy
its predecessor's saved settings.

## Build and upload

From the repository root in PowerShell:

```powershell
.\upload-imu.ps1 -BuildOnly
.\upload-imu.ps1
```

The shared upload helper targets **ESP32S3 Dev Module**, 4 MB flash, DIO 80 MHz,
hardware USB CDC/JTAG, USB CDC on boot, and disabled PSRAM. No PSRAM is needed.
The Adafruit GC9A01A, GFX, and BusIO libraries are required. Build files are kept
outside the repository. Close Serial Monitor before uploading.

To return to the sand simulation:

```powershell
.\upload-sand.ps1
```
