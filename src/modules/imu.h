#pragma once
// [FILL-IN] Pure IMU filtering. History is carried in ImuFilterState (no globals).

struct ImuRaw         { float gyro_z; float accel_x, accel_y; };
struct ImuFilterState { float yaw_rate_lp; bool seeded; };
struct ImuOutput      { float yaw_rate; float accel_x, accel_y; };

ImuOutput imu_compute(const ImuRaw &raw, ImuFilterState &s);
