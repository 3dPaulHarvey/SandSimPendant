# SandSimPendant

A tilt-responsive sand simulation for a circular display pendant, using a
Seeed XIAO ESP32-S3, GC9A01 display, CST816D touch controller, and MPU6050.

## Project contents

- [`Gravity_Sand`](Gravity_Sand/README.md): sand firmware, hardware wiring,
  controls, tuning, and build instructions.
- [`MPU_Visualizer`](MPU_Visualizer/README.md): motion visualization and shared
  flat-bias calibration.
- `Circlular_Touchscreen`: original touchscreen demo.
- `shared`: motion sensing, calibration, and shake detection.
- `tests`: native C++ checks for simulation, rendering, buttons, and motion.

Keep the sketch folders and `shared` together when cloning this repository.

## Build and upload

Install Arduino CLI (or Arduino IDE), the Espressif ESP32 board package, and the
Adafruit GC9A01A, Adafruit GFX, and Adafruit BusIO libraries. See the
[firmware documentation](Gravity_Sand/README.md) for board settings and wiring.

From the repository root in PowerShell:

```powershell
.\upload-sand.ps1 -BuildOnly
.\upload-sand.ps1 -Port COM23
```

Replace `COM23` with your board's port. The helper compiles before uploading and
stores intermediate build files outside the repository.

See [Code and checks](Gravity_Sand/README.md#code-and-checks) for native C++ tests.
The frozen reference engine used by the response test is included; generated
binaries, build outputs, and test images are ignored.
