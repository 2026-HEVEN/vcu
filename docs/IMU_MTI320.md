# MTi-320 integration contract

The VCU consumes Xsens Xbus `MTData2` (`MID 0x36`) at 115200 baud. Every
accepted frame must contain both of these Float32 data items:

- Acceleration (`XDI 0x4020`), converted from m/s² to g
- RateOfTurn (`XDI 0x8020`), with the Z axis converted from rad/s to deg/s

The firmware deliberately rejects checksum failures, malformed TLVs, extended
length frames, non-finite values, and frames missing either required item. It
keeps torque disabled until a complete sample arrives and enters the latched
Safety `Halt` state if valid samples stop for more than 100 ms while driving.

## Hardware requirement

MTi-320 exposes **RS-232**, not 3.3 V TTL UART. GPIO16/17 must be connected
through an RS-232-to-3.3 V UART transceiver. Direct connection can fail and may
electrically damage the ESP32 input.

## Sensor configuration requirement

Before vehicle operation, configure the MTi-320 with MT Manager or a validated
Xbus configuration tool to output the two items above as Float32 in every
MTData2 cycle at a rate comfortably faster than the 100 ms timeout. This
firmware does not overwrite the sensor's non-volatile output configuration at
boot; it proves the configuration at runtime by requiring both data items.

Record the selected coordinate frame and mounting transform in the harness
documentation. Confirm on the stationary vehicle and during a controlled
left/right rotation that positive Z RateOfTurn matches the sign convention
used by torque vectoring. Do not enable driving if this sign check fails.

## Bring-up checks

1. Confirm an RS-232 transceiver is fitted and TX/RX are crossed correctly.
2. Confirm 115200 baud, Xbus binary output, and MTData2 MID `0x36`.
3. Confirm Acceleration `0x4020` and RateOfTurn `0x8020` are present as Float32.
4. Confirm `imu_valid` remains true during motion and becomes false within
   100 ms after disconnecting the sensor.
5. Confirm a disconnect while driving sends zero torque and latches Safety
   `Halt`.
