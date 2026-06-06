#ifndef MECANUM_H
#define MECANUM_H

#include "sensor_packet.h"

typedef struct {
    float dx_body;
    float dy_body;
    float dtheta_wheel;
    float dtheta_gyro;
    float dtheta;
} OdometryDelta;

OdometryDelta mecanum_delta_from_packet(const SensorPacket *packet);

#endif
