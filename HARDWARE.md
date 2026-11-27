# Hardware catalog

Current selected hardware, updated 2026-11-27. Quantities below are per pendant;
the controller and screen product listings are each four-packs.

| Qty | Part | Selected hardware |
| --- | --- | --- |
| 1 | Controller | Aideepen ESP32-S3 Mini / Super Mini development board, USB-C, supplied without soldered headers; latest verified port **COM32** |
| 1 | Display | Standalone **1.28-inch GC9A01 round IPS TFT**, 240 x 240, seven-pin SPI module; **no touchscreen** |
| 1 | Motion sensor | **GY-521 development/breakout board sold as MPU6050**; replacement module reports nonstandard chip ID 0x75 and uses scoped compatibility mode |
| 2 | Buttons | Standard breadboard tactile pushbuttons, normally open and momentary; fluid explosion and new size/population controls |
| As needed | Connections/power | USB-C data/power cable, jumper wires, breadboard, and headers or soldered connections |

Battery, power switch, and enclosure are not yet selected.

## Controller: Aideepen ESP32-S3 Mini / Super Mini

Product identification supplied by the owner: **Aideepen 4-Pcs ESP32-S3 Mini
Development Board, ESP32-S3 Dual-Core WiFi + Bluetooth 5.0 IoT Dev Kit, USB-C,
240MHz MCU, 2.4GHz Wireless Module, Low Power Embedded Board (No Soldered Header)**.

| Specification | Value / evidence |
| --- | --- |
| Processor | Dual-core 32-bit Xtensa LX7, up to 240 MHz |
| Wireless | 2.4 GHz 802.11 b/g/n Wi-Fi and Bluetooth 5 LE |
| Internal SRAM | 512 KB |
| Flash | 4 MB embedded XMC flash, identified on this board by esptool |
| PSRAM | 2 MB embedded PSRAM, identified by esptool; runtime memory operation not yet tested |
| Chip revision | ESP32-S3 revision 0.2, observed during diagnostics |
| USB | USB-C; native USB Serial/JTAG, VID 303A / PID 1001; last verified on COM32 |
| Onboard controls | BOOT and RESET buttons |
| Onboard RGB LED | Addressable RGB LED on GPIO48; red/green/blue operation verified |
| Arduino board setting | ESP32S3 Dev Module; see the diagnostic README for tested settings |

Processor, SRAM, and radio specifications are verified against
[Espressif's ESP32-S3 overview](https://www.espressif.com/en/products/socs/esp32-s3/).
The detected flash/PSRAM combination matches the ESP32-S3FH4R2 entry in
[Espressif's datasheet](https://documentation.espressif.com/esp32-s3_datasheet_en.pdf).
Board-specific observations are recorded in
[`ESP32S3_Diagnostics/README.md`](ESP32S3_Diagnostics/README.md); the saved
`build/ESP32S3_Diagnostics/upload.log` and heartbeat logs contain the local evidence.

The supplied board diagram also depicts B+/B- battery pads and a charging-current
jumper. Those features have not been electrically verified on this board; battery
selection and charging configuration remain open.

## Display: standalone GC9A01 SPI module

The owner supplied the product specifications and front/back photos. These are
supplier-listed dimensions and ratings, not measurements of the physical module.

| Specification | Listed value |
| --- | --- |
| Panel | 1.28-inch round IPS TFT, full-color RGB |
| Resolution / driver | 240 x 240 pixels / GC9A01 |
| Interface | Four-wire SPI; seven header pins including power and reset |
| PCB dimensions | 38 x 45.5 x 3.2 mm |
| Display area | 32.4 x 32.4 mm |
| Viewing angle | 178 degrees, supplier claim |
| Operating temperature | -20 to +70 degrees C, supplier rating |
| Supply | Advertised 3-5 V; project plan is 3.3 V supply and 3.3 V GPIO signals |
| Touch | None |

Pin numbering follows the supplied interface definition. Match the printed pin
labels when viewing the module from the opposite side.

| Pin | Label | Function |
| --- | --- | --- |
| 1 | RST | Display reset |
| 2 | CS | SPI chip select |
| 3 | DC | Data/command selection |
| 4 | SDA | SPI MOSI data input, not I2C SDA |
| 5 | SCL | SPI clock (SCK), not I2C SCL |
| 6 | GND | Ground |
| 7 | VCC | Module power |

There is no separate backlight pin on this seven-pin header. The advertised
supply range does not establish 5 V tolerance for signal inputs.

## Motion sensor and buttons

- The motion sensor is the **GY-521 module**, not a bare MPU6050 chip. It uses
  I2C; the driver supports addresses 0x68 and 0x69. The installed replacement
  responds at 0x68 but reports identity 0x75; its exact chip model is unconfirmed.
- The two buttons are ordinary breadboard tactile switches, not capacitive
  touch controls. Exact dimensions and manufacturer are unspecified.
- In the fluid sketch, the primary button triggers an explosion and the
  secondary button creates a new dot size/population. Reset and palette selection
  remain available over serial. The visualizer uses these same switches for
  calibration and response mode.

## Project wiring

These are raw GPIO numbers on the Super Mini. Both application sketches use
[`shared/BoardConfig.h`](shared/BoardConfig.h) as the single pin configuration.
The owner accepted this wiring map on 2026-11-22.

The secondary button was subsequently moved from GPIO8 to **GPIO11** at the
owner's request. Move only its signal lead; the other switch contact remains
connected to GND. Both sketches share this assignment in `BoardConfig.h`.

| Module connection | Super Mini connection |
| --- | --- |
| LCD RST | GPIO4 |
| LCD CS | GPIO1 |
| LCD DC | GPIO2 |
| LCD SDA / MOSI | GPIO9 |
| LCD SCL / SCK | GPIO7 |
| LCD VCC / GND | 3V3 / GND |
| GY-521 SDA | GPIO12 |
| GY-521 SCL | GPIO13 |
| GY-521 VCC / GND | 3V3 / GND |
| Primary button (fluid explosion) | GPIO10 to GND |
| Secondary button (new fluid population) | GPIO11 to GND |

The GY-521 bus moved to GPIO12 (SDA) and GPIO13 (SCL) on 2026-11-23.
GPIO5 and GPIO6 are now unused by the project.
See the [annotated board diagram](assets/wiring/esp32-s3-wiring-gpio12-13-v2.png)
for the current connections.

Leave GY-521 INT and auxiliary XDA/XCL unconnected. AD0 may remain at its default
low level (0x68); the driver also accepts 0x69. Each button uses an internal pull-up.
On a four-leg tactile switch, the two legs on each internally connected side are
one contact; wire across the two contacts, which join only when pressed.
Both switches are normally open and momentary. Firmware uses 25 ms debounce,
fires once per press, and ignores a switch held at boot until it is released.

The initial display SPI rate is 20 MHz. The shared GY-521 I2C default and the
visualizer use 400 kHz; the fluid test overrides this to 100 kHz in `SandConfig.h`
and provides serial `b` to compare 100/400 kHz. This clock choice does not establish
bus reliability. GPIO48 stays
reserved for the onboard RGB LED; native USB uses GPIO19/20. The selected
external pins avoid strapping GPIO3. All firmware targets **ESP32S3 Dev Module**
with the board's 4 MB flash and PSRAM disabled.

## Hardware verification status

Board USB, RGB LED, flash upload, and power-cycle startup were verified in the
original diagnostic. The screen/IMU test is `MPU_Visualizer`; its live telemetry
and visual check establish whether the new external modules are working. A
successful SPI write alone cannot confirm what the physical display shows.

Replacement-module checks on 2026-11-22:

- Owner confirmed the physical display works.
- Replacement GY-521 acknowledged at 0x68 at both 100 and 400 kHz. Identity
  register 0x75 consistently returned **0x75** with both repeated-start and
  STOP-separated reads. Standard MPU6050 identity is **0x68**, according to
  [TDK's register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf).
- The module returned changing 14-byte measurement blocks. Configuration
  write/readback checks passed, followed by live samples at about **111 Hz**,
  display-loop rate about **18.5 FPS**, and **zero I2C errors** in the captured run.
- Acceleration was approximately (0.22, 0.00, 0.93)g, magnitude about 0.96g;
  readings are uncalibrated. These observations support basic register
  compatibility, not identification of a specific chip or validation of all
  MPU6050 filter/scale specifications.

`kImuCompatibilityId` in `shared/BoardConfig.h` allows only this observed 0x75
identity in addition to standard 0x68. Set it to zero for strict identification.
The driver still requires configuration readback and fresh data. The visualizer
labels this mode `ID75 COMPAT`; other unknown IDs are rejected. Probe, register,
and live logs are saved under `build/MPU_Visualizer/replacement-*.log`.

## Replacement ESP32 controller, 2026-11-22

The replacement controller enumerates as **COM32**, USB Serial/JTAG VID 303A /
PID 1001, MAC `98:c3:77:63:d3:18`. It is an ESP32-S3 revision 0.2 with 4 MB XMC
flash and 2 MB embedded PSRAM; the application continues to leave PSRAM disabled.

Before flashing, its ROM repeatedly printed `invalid header: 0xffffffff` and
`TG0WDT_SYS_RST` while attempting `SPI_FAST_FLASH_BOOT`. Reading the first
4,096 bytes of flash at address 0x0 returned only 0xFF: the bootloader sector
was erased. Flash identification/readback succeeded, and Secure Boot and flash
encryption were disabled. This supports a missing boot image as the cause of
the initial boot loop. The S3 bootloader belongs at offset 0x0, as documented
in [Espressif's troubleshooting guide](https://docs.espressif.com/projects/esptool/en/latest/esp32s3/troubleshooting.html#missing-bootloader).

The current Gravity_Sand bootloader, partition table, boot-app metadata, and
application were uploaded; all segments passed hash verification. The board
restarted into the application and completed a 36-second live serial check
without a USB disconnect or observed reboot. Display telemetry stayed around
60 FPS, physics around 240 Hz; serial `n` and `e` both worked. The check ended
at the default 100 dots with live tilt enabled. GPIO8 still read LOW and the
IMU still reported rejected samples; replacing the controller did not remove
those separate symptoms.

Two additional serial-monitor connections with DTR/RTS left deasserted preserved
the running counters across close/reopen. GPIO10 physical explosion presses were
recorded; GPIO8 remained LOW with input and pull-up enabled and output disabled.
The checks distinguish successful controller recovery from the unresolved
peripheral signals; they do not establish that the button circuit or IMU bus is
healthy.

Upload scripts now detect a single present Espressif USB serial port rather
than assuming COM31. With zero or multiple matches they stop and request an
explicit `-Port`; an explicit port is never silently redirected. Evidence is
saved in `build/Gravity_Sand/replacement-*` and the upload log (ignored by Git).

The GPIO11 remap was then built and uploaded to COM32 with all flash segments
verified. Live telemetry recorded five accepted population-button presses and
a change to seven dots at 327.7% size. The pin configuration confirmed input
enabled, output disabled, and the internal pull-up enabled. Its sampled state
afterward was LOW; press registration alone does not verify a clean released
switch state. This capture is `build/Gravity_Sand/gpio11-verification.log`.

Resolution confirmed by the owner on 2026-11-22: the GPIO problems were caused
by bad wiring, and the GPIOs/buttons now work correctly after the wiring fix.
The earlier LOW readings above are historical observations. The current button
assignments remain GPIO10 for explosions and GPIO11 for size/count, both with
`INPUT_PULLUP` and switches to GND.

The owner further explained that the buttons had been inserted into breadboard
rails connected to the IMU. Measurements taken with that wiring cannot isolate
IMU hardware health from button-induced electrical interference. The recorded
I2C errors and rejected samples may therefore have been wiring effects; they
are not evidence by themselves of a defective IMU. Keep the earlier identity
observation and the independently reproduced software drift/damping fixes
separate from this wiring diagnosis.

A fresh passive 30-second capture on COM32 after the reported wiring correction
showed both button inputs HIGH. Across 15 IMU reports, the cumulative driver
error count increased from 4,873 to 5,388 and rejected samples from 3,408 to
3,807 (515 errors, including 399 rejected samples). Counters did not reset;
motion data stayed ready, but shake qualification remained false. The display
remained around 60 FPS. These new errors show that an IMU data issue remains in
this observation window; they do not identify whether its cause is wiring,
power, sensor compatibility, or the sensor itself. No firmware or calibration
settings were changed. Evidence: `build/Gravity_Sand/corrected-wiring-imu.log`
and `corrected-wiring-imu-summary.json` (ignored by Git).

## GY-521 GPIO12/13 verification, 2026-11-23

Gravity_Sand was rebuilt with SDA on GPIO12 and SCL on GPIO13 and uploaded
to COM32; all four flash segments passed hash verification. MPU_Visualizer
also compiles with the shared mapping. After the owner finished rewiring,
a 36-second serial capture found the sensor at 0x68 with its existing 0x75
identity using both repeated-start and STOP-separated reads.

At the default 100 kHz I2C clock, accepted samples ran about 180-185 Hz.
Every reported driver-error and rejected-sample count was zero; motion-ready
and shake-trusted flags remained true. Software rendering stayed about 60 FPS,
physics about 240 Hz, and free heap stayed at 95,940 bytes. No crash or reboot
was observed. Calibration remains unset; the existing scoped 0x75 compatibility
mode is unchanged.

The earlier sample errors did not recur in this capture. This verifies the
current wiring in the observed interval; it does not isolate the GPIO change
from other wiring changes or identify the sensor as a genuine MPU6050.
Evidence: `build/imu-gpio12-13-COM32/live-verification.log` and
`verification-summary.json` (ignored by Git).

COM33 (MAC `98:c3:77:63:d5:94`) was subsequently checked with its newly wired
GY-521. Its installed firmware still used GPIO5/6, so the current GPIO12/13
build was uploaded; all four segments verified. A 36-second live check then
found address 0x68 and the same 0x75 compatibility identity, delivering about
182-188 accepted samples per second with zero driver errors or rejected
samples. Motion-ready and shake-trusted flags stayed true. Software rendering
was about 60 FPS, physics about 240 Hz, and heap stayed at 95,896 bytes with no
observed crash or reboot. Calibration remains unset. Evidence is saved in
`build/imu-gpio12-13-COM33/live-verification.log` and `verification-summary.json`.

## IMU mounting direction

The earlier direction report established a quarter-turn mismatch: left tilt
moved the dot up, toward moved it left, right moved it down, and away moved it
right. The correction maps the previous screen vector `(x,y)` to `(y,-x)`.
That earlier chip-to-screen mapping was **(+accelY, +accelX)**, represented by
**swap=1, flipX=1, flipY=0** and saved on COM31.

The new COM33 prototype reverses both directions according to the owner's
subsequent report. Its correction is **(-accelY, -accelX)**:
**swap=1, flipX=0, flipY=1**. These became the shared defaults at that time;
the saved COM33 settings were also reversed. Both sketches
load saved mounting settings ahead of defaults, and directional shake uses
the same mapping as tilt. Bias calibration and the displayed physical
chip-axis measurements are independent and are unchanged by this correction.

COM33 acknowledged both reversals before upload. With similar raw acceleration
near `(0.41, 0.18, 0.84)g`, screen gravity changed from approximately
`(0.18, 0.41)` to `(-0.18, -0.41)`. After all four firmware segments verified
and the board restarted, diagnostics twice reported
`MOUNT swap=1 flipX=0 flipY=1 saved=1`; `saved=1` checks that all three NVS
keys exist and match the active mapping. The 20-second post-upload capture had
zero driver errors and rejected samples and about 60 FPS. Both sketches build,
and the native motion-calibration/direction suite passes. Evidence is in
`build/mount-180-COM33` (ignored by Git).

## COM35 face-flipped IMU mount, 2026-11-26

The owner turned the sensor over while keeping its connector end at the top
of the assembly. Live readings had Z around -0.88g, consistent with the flip.
With that connector orientation, the inferred correction reverses screen X
relative to COM33 and retains screen Y: **(+accelY, -accelX)**, or
**swap=1, flipX=1, flipY=1**. This is now the shared default and was saved on
COM35 (MAC `90:da:72:72:ab:48`). Raw sensor axes and bias calibration were
not changed; tilt and directional shake share the corrected mapping.

The display was not yet wired, so the owner requested this best estimate.
Physical left/right and up/down response remains to be checked after wiring.

All four Gravity_Sand flash segments verified. Two post-restart diagnostic
reports confirmed `MOUNT swap=1 flipX=1 flipY=1 saved=1`. A 30-second capture
showed 183.0-187.4Hz IMU samples, zero driver errors and rejected samples,
software rendering at 59.5-60.5 FPS, and no observed reboot. Both application
sketches build; the five focused mapping, driver, and shake tests pass.
Evidence is saved in `build/mount-flipped-COM35` (ignored by Git).

## Tilt color-wheel LED, COM32, 2026-11-27

Gravity_Sand now drives the onboard GPIO48 RGB LED from filtered, screen-mapped
tilt. The IMU visualizer shares the same LED behavior. No additional wiring or
libraries are required; the standalone diagnostic retains its RGB test cycle.
See the [LED mapping and tuning](Gravity_Sand/README.md#tilt-color-wheel-led).

The connected COM32 board (MAC `98:c3:77:63:d3:18`) received the new firmware;
all four flash segments verified. Active mount defaults were
`swap=1 flipX=1 flipY=1`, with no saved mapping reported. A near-level capture
confirmed RGB output off, no repeated LED writes, roughly 60 software FPS, and
zero observed IMU errors. Physical wheel direction and color appearance await a
tilt check; no calibration or mount preferences were written by this change.
Logs are in `build/tilt-rgb/` (ignored by Git).
