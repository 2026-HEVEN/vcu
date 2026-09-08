// Links the real VCU encoder and real Cluster decoder/receiver in one process.
#include "car_check_protocol.h"
#include "car_check_receiver.h"
#include <cassert>
#include <cmath>
#include <cstdio>
int main() {
    CarCheckReceiver rx; uint8_t d[8];
    for(int n=-1000;n<=1000;++n) {
        car_check::Steering s; s.unit=float(n)/1000; s.valid=true; s.life=uint8_t(n);
        car_check::encode_steering(s,d);
        assert(rx.receive(car_check::STEERING_ID,true,8,d,uint32_t(n+1000)));
        assert(rx.steering_quality(uint32_t(n+1000))==car_check::Quality::Valid);
        assert(std::fabs(rx.steering.unit-s.unit)<.0011f);
    }
    for(int mask=0;mask<16;++mask) {
        car_check::Wheels w;
        for(int i=0;i<4;++i) { w.kph[i]=i*12.3f; w.valid[i]=(mask&(1<<i))!=0; }
        car_check::encode_wheels(w,d);
        assert(rx.receive(car_check::WHEELS_ID,true,8,d,3000));
        for(int i=0;i<4;++i) {
            assert(rx.wheels.valid[i]==w.valid[i]);
            if(w.valid[i]) assert(std::fabs(rx.wheels.kph[i]-w.kph[i])<.051f);
        }
    }
    for(int mask=0;mask<4;++mask) {
        car_check::Imu m; m.yaw_dps=-123.45f; m.ax_g=-1.23f; m.ay_g=2.34f;
        m.yaw_valid=mask&1; m.accel_valid=mask&2; m.life=255;
        car_check::encode_imu(m,d);
        assert(rx.receive(car_check::IMU_ID,true,8,d,3000));
        assert(rx.imu.yaw_valid==m.yaw_valid && rx.imu.accel_valid==m.accel_valid);
        if(m.yaw_valid) assert(std::fabs(rx.imu.yaw_dps-m.yaw_dps)<.011f);
        if(m.accel_valid) assert(std::fabs(rx.imu.ay_g-m.ay_g)<.011f);
    }
    for(int flags=0;flags<128;++flags) {
        car_check::Control c;
        c.tv_requested=c.regen_requested=c.paddock_requested=true;
        c.tv_active=flags&1; c.regen_available=flags&2; c.regen_active=flags&4;
        c.paddock_active=flags&8; c.output_allowed=flags&16;
        c.cluster_fresh=flags&32; c.brake_installed=flags&64;
        c.tv_block=0x7f; c.regen_block=0xff; c.life=42;
        car_check::encode_control(c,d);
        assert(rx.receive(car_check::CONTROL_ID,true,8,d,3000));
        const auto &r=rx.control;
        assert(r.supported && r.tv_requested && r.regen_requested && r.paddock_requested);
        assert(r.tv_active==c.tv_active && r.regen_available==c.regen_available);
        assert(r.regen_active==c.regen_active && r.paddock_active==c.paddock_active);
        assert(r.output_allowed==c.output_allowed && r.cluster_fresh==c.cluster_fresh);
        assert(r.brake_installed==c.brake_installed && r.tv_block==0x7f && r.regen_block==0xff);
        assert(r.life==42);
    }
    assert(rx.control_quality(3300)==car_check::Quality::Valid);
    assert(rx.control_quality(3301)==car_check::Quality::Stale);
    std::puts("PASS: actual VCU encoder -> actual Cluster receiver (2149 vectors + freshness)");
}
