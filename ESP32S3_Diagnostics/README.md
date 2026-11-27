# ESP32-S3 Super Mini board diagnostic

Optional onboard RGB/USB check for the Aideepen board. For the external screen
and GY-521, use the [IMU visualizer](../MPU_Visualizer/README.md).

## Operation

The onboard addressable RGB LED on GPIO48 cycles red, off, green, off, blue,
off, with one second per step. Serial telemetry at 115200 baud reports uptime,
reset reason, flash size, and free heap. The sketch never waits for a serial
monitor. It uses the shared board configuration and leaves external peripheral
pins alone.

```powershell
.\ESP32S3_Diagnostics\upload.ps1 -BuildOnly
.\ESP32S3_Diagnostics\upload.ps1
```

The wrapper uses the same generic ESP32S3 Dev Module settings as the main
project in `upload-firmware.ps1`: 240 MHz, 4 MB flash, DIO, default partitions,
PSRAM disabled, and native USB CDC enabled. Close Serial Monitor before upload.
The uploader detects a single connected Espressif serial port; with multiple
boards attached, select one explicitly with `-Port COM32` (or its actual port).
If automatic connection fails, hold BOOT, press/release RESET, then release BOOT
and retry. The uploader uses a watchdog reset to start the sketch afterward.

## Recorded board checks, 2026-11-22

- COM31, USB Serial/JTAG (VID 303A / PID 1001).
- ESP32-S3 revision 0.2; embedded 4 MB XMC flash and 2 MB PSRAM identified.
- Firmware segments passed esptool hash verification.
- Initial 12-second serial capture showed increasing uptime and constant free heap.
- Owner confirmed RGB colors and automatic restart after USB unplug/reconnect.
- PSRAM was detected but disabled; runtime PSRAM operation was not tested.

## Saved evidence

The original diagnostics and backup remain in the ignored
`build/ESP32S3_Diagnostics` folder. `before-diagnostics-COM31.bin` is the 4 MB
flash backup from before the initial test; it is not necessarily factory firmware.
`heartbeat.log` and `power-cycle-heartbeat.log` record the original tests.
Those historical logs also mention a GPIO2 test output from the initial sketch;
that output was removed because GPIO2 is now assigned to the LCD DC input.
New builds and uploads update `compile.log` and `upload.log` in that folder.
