#include "mecanum.h"

#include "estimator_config.h"

OdometryDelta mecanum_delta_from_packet(const SensorPacket *packet) {
    const float l = EST_LX_M + EST_LY_M;
    const float fl = packet->front_left_ticks * EST_METER_PER_TICK;
    const float fr = packet->front_right_ticks * EST_METER_PER_TICK;
    const float rl = packet->rear_left_ticks * EST_METER_PER_TICK;
    const float rr = packet->rear_right_ticks * EST_METER_PER_TICK;

    OdometryDelta odom;
    odom.dx_body = (fl + fr + rl + rr) * 0.25f;
    odom.dy_body = (-fl + fr + rl - rr) * 0.25f;
    odom.dtheta_wheel = (-fl + fr - rl + rr) / (4.0f * l);
    odom.dtheta_gyro = packet->gyro_z * EST_DT_SEC;
    odom.dtheta =
        EST_GYRO_HEADING_WEIGHT * odom.dtheta_gyro +
        EST_WHEEL_HEADING_WEIGHT * odom.dtheta_wheel;
    return odom;
}
