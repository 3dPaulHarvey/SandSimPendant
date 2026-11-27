# Firmware checkpoints

## Pre-effects baseline — 2026-11-27

Git tag: **`pre-effects-2026-11-27`**

This checkpoint preserves the current pendant program before adding further
special effects. It includes the existing moving gradient, velocity stretch,
wall-impact squash, and one alternate-shape particle. Checkpoint preparation
corrects documentation and trims stray whitespace; firmware behavior is preserved.

### Saved program

- ESP32-S3 Super Mini, GC9A01 240 × 240 display at 20 MHz SPI, and GY-521 on
  GPIO12/13. Explosion and Randomize buttons use GPIO10/11.
- Circle, triangle, square, and star populations; 100–350% preferred visual
  sizes and approximately 15–35% spawn fill. Startup targets 22% fill near
  a preferred 300% size.
- 21 color swatches, five foreground colors per swatch, and the animated
  three-layer background. Exactly one particle uses an alternate shape.
- Explosions sample 600–1,200 px/s. Physics targets 240 Hz; the independent
  LCD task targets 60 snapshots per second.
- Shared motion calibration, input-health checks, shake processing, and the
  current COM35 mounting defaults: `swap=1`, `flipX=1`, `flipY=1`.
- IMU visualizer, board diagnostics, upload scripts, native tests, palette
  data, plotting tools, and wiring documentation.

### Validation performed for this checkpoint

- All **24 native test suites passed**.
- Six additional suites passed with `ARDUINO_ARCH_ESP32` enabled: renderer,
  shapes, background, render effects, coverage, and population. These exercise
  the scoped optimization paths on desktop GCC, not device performance.
- All three sketches compiled successfully with the repository upload scripts
  in `-BuildOnly` mode. No firmware was uploaded during checkpoint preparation.

| Sketch | Program storage | Static RAM |
| --- | ---: | ---: |
| Gravity_Sand | 427,042 bytes | 250,856 bytes (76%) |
| MPU_Visualizer | 360,730 bytes | 29,584 bytes (9%) |
| ESP32S3_Diagnostics | 297,618 bytes | 22,320 bytes (6%) |

Gravity_Sand retains the compiler's low-memory warning, with 76,824 bytes
remaining in its reported static RAM budget before runtime allocations.
The 60 FPS rate is a scheduling target: earlier busy explosion tests exceeded
the frame budget. The latest COM35 mounting direction still needs a physical
tilt check with the display connected; see [hardware notes](HARDWARE.md).

Build environment: Arduino CLI 1.6.0, Espressif ESP32 core 3.3.12,
Adafruit GC9A01A 1.2.0, Adafruit GFX 1.13.0, and Adafruit BusIO 1.17.4.
The board settings remain in `upload-firmware.ps1` (240 MHz CPU, 4 MB flash,
DIO, default partitions, PSRAM disabled, native USB CDC, loop on core 1).

### Retrieve and rebuild

From a clone of this repository, create a separate checkout of the saved tag:

```powershell
git fetch origin --tags
git worktree add -b restore/pre-effects ../SimCircle-pre-effects pre-effects-2026-11-27
Set-Location ../SimCircle-pre-effects
.\tests\run-native-tests.ps1
.\upload-sand.ps1 -BuildOnly
```

Use a different branch and directory name if those already exist. Board-stored
calibration and mounting preferences are separate from Git and override defaults;
the checkpoint does not back up device NVS.

Local validation logs, compiled firmware copies, a source ZIP, and a verified
Git bundle are retained under `build/checkpoint-2026-11-27/`. These local recovery
artifacts and historical telemetry remain ignored; GitHub stores the source
checkpoint and tag.
