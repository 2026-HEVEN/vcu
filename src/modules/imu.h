#pragma once
// Pure IMU mapping. MTi-320 produces calibrated Acceleration and RateOfTurn;
// Xbus framing, unit conversion, and freshness checks happen outside this API.

struct ImuRaw    { float yaw_rate; float accel_x, accel_y; };
struct ImuOutput { float yaw_rate; float accel_x, accel_y; };

ImuOutput imu_compute(const ImuRaw &raw);
