#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <stdbool.h>
#include "mecanum.h"
#include "sensor_packet.h"

typedef struct {
    float x;
    float y;
    float theta;
} Pose2D;

void pf_init(float x, float y, float theta);
void pf_predict(const OdometryDelta *odom);
void pf_update_uwb(const SensorPacket *packet);
void pf_resample_if_needed(void);
Pose2D pf_estimate(void);
float pf_effective_sample_size(void);

#endif
