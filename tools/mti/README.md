# MTi-320 inclination zero through ESP32

This is **not factory reset**. It aligns current roll/pitch and explicitly stores
the alignment in the sensor. It does not change baud, output configuration or
firmware, and does not reset heading or compensate IMU position relative to CG.

## Tasks

1. HV/motor power OFF. Close MT Manager and all serial monitors.
2. PlatformIO Project Tasks → `mti_bridge` → Upload. This replaces VCU firmware;
   bridge has no CAN/motor control. RX=GPIO19, TX=GPIO21, 115200 8N1 through RS232
   level converter. Never connect RS232 voltage directly to ESP32.
3. Run custom target `mti_check` or `mti_zero` in Project Tasks. If the extension
   does not refresh, use **PlatformIO: Refresh Project Tasks** / reload window.
   CLI in PlatformIO terminal: `pio run -e mti_bridge -t mti_zero`.
4. Select the actual COM port (not hardcoded COM13). For zero, vehicle must be
   level, stationary and sensor firmly mounted. Enter `ZERO`, then displayed
   sensor ID. Keep vehicle still throughout. ACK, sample validity and motion
   checks must pass before storing. Missing ACK stops; no blind retries.
5. Power-cycle the MTi and run `mti_check` again; near-zero roll/pitch and X/Y
   acceleration confirm persistence. Check does enter config/measurement, but
   does not alter alignment. Z acceleration near 9.81 m/s² is expected.
6. **Restore `esp32dev` → Upload before driving.** No automatic restore/upload.

VS Code `Tasks: Run Task` also exposes the four numbered MTi tasks. Those require
`platformio` on the shell PATH; otherwise use the PlatformIO Project Tasks or its
terminal. No developer-specific absolute paths are required. Optional local port:
`python tools/mti/calibrate.py check --port COM13` (PlatformIO Python has pyserial).

`zero` uses measurement-mode ResetOrientation(0xA4, 0x0003), verifies fresh
float32 ENU Euler/Acceleration/RateOfTurn samples, enters config, stores with
0xA4/0x0000, and returns to measurement. Store ACK is not power-cycle proof.
If interrupted after store, alignment may already be persistent: inspect rather
than assuming rollback. No factory reset, gyro bias calibration or VCU changes.

Mounting alignment and the VCU X/Y sign convention are separate; after changes,
verify forward acceleration and left-turn signs before enabling TV. Gravity
alignment cannot establish vehicle heading or prove the floor is level.

Protocol: https://www.xsens.com/hubfs/Downloads/Manuals/MT_Low-Level_Documentation.pdf
