#include "car_check_protocol.h"
#include <cmath>

namespace car_check {
namespace {
void clear(uint8_t out[8]) { for(int i=0;i<8;++i) out[i]=0; }
void put16(uint8_t *p, uint16_t v) { p[0]=uint8_t(v); p[1]=uint8_t(v>>8); }
bool fits(float v, float scale) {
    return std::isfinite(v) && v*scale >= -32768.0f && v*scale <= 32767.0f;
}
void signed16(uint8_t *p, float v, float scale) {
    const float x=v*scale;
    put16(p, uint16_t(int16_t(x>=0 ? x+0.5f : x-0.5f)));
}
}
void encode_steering(const Steering &in, uint8_t out[8]) {
    clear(out);
    const bool valid=in.valid && std::isfinite(in.unit) && std::fabs(in.unit)<=1.0f;
    if(valid) signed16(out,in.unit,1000.0f);
    out[6]=VALIDITY_PRESENT | (valid ? 1 : 0);
    out[7]=in.life;
}
void encode_imu(const Imu &in, uint8_t out[8]) {
    clear(out);
    const bool yaw=in.yaw_valid && fits(in.yaw_dps,100.0f);
    const bool accel=in.accel_valid && fits(in.ax_g,100.0f) && fits(in.ay_g,100.0f);
    if(yaw) signed16(out,in.yaw_dps,100.0f);
    if(accel) { signed16(out+2,in.ax_g,100.0f); signed16(out+4,in.ay_g,100.0f); }
    out[6]=VALIDITY_PRESENT | (yaw ? 1 : 0) | (accel ? 2 : 0);
    out[7]=in.life;
}
void encode_wheels(const Wheels &in, uint8_t out[8]) {
    for(int i=0;i<4;++i) {
        const float v=in.kph[i];
        const bool valid=in.valid[i] && std::isfinite(v) && v>=0 && v<=6553.4f;
        put16(out+2*i,valid ? uint16_t(v*10.0f+0.5f) : INVALID_WHEEL);
    }
}
void encode_control(const Control &in, uint8_t out[8]) {
    clear(out); out[0]=VERSION;
    out[1]=(in.tv_requested?1:0)|(in.regen_requested?2:0)|(in.paddock_requested?4:0);
    out[2]=(in.tv_active?1:0)|(in.regen_available?2:0)|(in.regen_active?4:0)|
        (in.paddock_active?8:0)|(in.output_allowed?16:0)|(in.cluster_fresh?32:0)|
        (in.brake_installed?64:0);
    out[3]=in.tv_block; out[4]=in.regen_block; out[7]=in.life;
}
}
