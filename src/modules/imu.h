#pragma once
// [FILL-IN] Pure IMU mapping. MTi-320 runs its own AHRS fusion on-device, so
// yaw_rate/accel arrive already bias-corrected and drift-free — no filter
// state left to carry between calls.

struct ImuRaw    { float yaw_rate; float accel_x, accel_y; };
struct ImuOutput { float yaw_rate; float accel_x, accel_y; };

ImuOutput imu_compute(const ImuRaw &raw);
