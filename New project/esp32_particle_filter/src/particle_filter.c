#include "particle_filter.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "estimator_config.h"

typedef struct {
    float x;
    float y;
    float theta;
    float weight;
} Particle;

static Particle particles[EST_PARTICLE_COUNT];
static Particle scratch[EST_PARTICLE_COUNT];

static float rand_unit(void) {
    return ((float)rand() + 1.0f) / ((float)RAND_MAX + 2.0f);
}

static float rand_normal(float stddev) {
    const float u1 = rand_unit();
    const float u2 = rand_unit();
    const float r = sqrtf(-2.0f * logf(u1));
    const float theta = 2.0f * 3.14159265358979323846f * u2;
    return stddev * r * cosf(theta);
}

static float wrap_pi(float angle) {
    while (angle > 3.14159265358979323846f) {
        angle -= 2.0f * 3.14159265358979323846f;
    }
    while (angle < -3.14159265358979323846f) {
        angle += 2.0f * 3.14159265358979323846f;
    }
    return angle;
}

static float clampf_local(float value, float low, float high) {
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static void normalize_weights(void) {
    float sum = 0.0f;

    for (int i = 0; i < EST_PARTICLE_COUNT; i++) {
        sum += particles[i].weight;
    }

    if (sum <= 0.0f || !isfinite(sum)) {
        const float uniform = 1.0f / (float)EST_PARTICLE_COUNT;
        for (int i = 0; i < EST_PARTICLE_COUNT; i++) {
            particles[i].weight = uniform;
        }
        return;
    }

    for (int i = 0; i < EST_PARTICLE_COUNT; i++) {
        particles[i].weight /= sum;
    }
}

void pf_init(float x, float y, float theta) {
    const float uniform = 1.0f / (float)EST_PARTICLE_COUNT;

    srand(7);
    for (int i = 0; i < EST_PARTICLE_COUNT; i++) {
        particles[i].x = clampf_local(x + rand_normal(EST_INIT_POS_STD_M), 0.0f, EST_MAP_WIDTH_M);
        particles[i].y = clampf_local(y + rand_normal(EST_INIT_POS_STD_M), 0.0f, EST_MAP_HEIGHT_M);
        particles[i].theta = wrap_pi(theta + rand_normal(EST_INIT_THETA_STD_RAD));
        particles[i].weight = uniform;
    }
}

void pf_predict(const OdometryDelta *odom) {
    for (int i = 0; i < EST_PARTICLE_COUNT; i++) {
        const float dx = odom->dx_body + rand_normal(EST_ODOM_X_STD_M);
        const float dy = odom->dy_body + rand_normal(EST_ODOM_Y_STD_M);
        const float dtheta = odom->dtheta +
            rand_normal(EST_GYRO_THETA_STD_RAD + EST_ODOM_THETA_STD_RAD);
        const float theta_mid = particles[i].theta + 0.5f * dtheta;
        const float c = cosf(theta_mid);
        const float s = sinf(theta_mid);

        particles[i].x += c * dx - s * dy;
        particles[i].y += s * dx + c * dy;
        particles[i].theta = wrap_pi(particles[i].theta + dtheta);

        particles[i].x = clampf_local(particles[i].x, 0.0f, EST_MAP_WIDTH_M);
        particles[i].y = clampf_local(particles[i].y, 0.0f, EST_MAP_HEIGHT_M);
    }
}

void pf_update_uwb(const SensorPacket *packet) {
    if (packet->uwb_count <= 0) {
        return;
    }

    for (int i = 0; i < EST_PARTICLE_COUNT; i++) {
        float log_likelihood = 0.0f;
        const float c = cosf(particles[i].theta);
        const float s = sinf(particles[i].theta);
        const float tag_x = particles[i].x + c * EST_TAG_OFFSET_X_M - s * EST_TAG_OFFSET_Y_M;
        const float tag_y = particles[i].y + s * EST_TAG_OFFSET_X_M + c * EST_TAG_OFFSET_Y_M;
        const float tag_z = EST_TAG_OFFSET_Z_M;

        for (int j = 0; j < packet->uwb_count; j++) {
            const UwbMeasurement *m = &packet->uwb[j];
            const float dx = tag_x - m->x;
            const float dy = tag_y - m->y;
            const float dz = tag_z - m->z;
            const float predicted = sqrtf(dx * dx + dy * dy + dz * dz);
            const float stddev = fmaxf(m->std, 0.03f);
            const float residual = m->distance - predicted;
            log_likelihood += -0.5f * (residual * residual) / (stddev * stddev);
        }

        particles[i].weight *= expf(log_likelihood);
    }

    normalize_weights();
}

float pf_effective_sample_size(void) {
    float sum_sq = 0.0f;

    for (int i = 0; i < EST_PARTICLE_COUNT; i++) {
        sum_sq += particles[i].weight * particles[i].weight;
    }

    if (sum_sq <= 0.0f) {
        return 0.0f;
    }
    return 1.0f / sum_sq;
}

void pf_resample_if_needed(void) {
    const float neff = pf_effective_sample_size();
    const float threshold = EST_RESAMPLE_NEFF_RATIO * (float)EST_PARTICLE_COUNT;
    float cumulative = 0.0f;
    float r;
    int index = 0;

    if (neff >= threshold) {
        return;
    }

    r = rand_unit() / (float)EST_PARTICLE_COUNT;

    for (int m = 0; m < EST_PARTICLE_COUNT; m++) {
        const float u = r + (float)m / (float)EST_PARTICLE_COUNT;
        while (index < EST_PARTICLE_COUNT - 1 &&
               cumulative + particles[index].weight < u) {
            cumulative += particles[index].weight;
            index++;
        }
        scratch[m] = particles[index];
        scratch[m].weight = 1.0f / (float)EST_PARTICLE_COUNT;
    }

    memcpy(particles, scratch, sizeof(particles));
}

Pose2D pf_estimate(void) {
    Pose2D pose = {0.0f, 0.0f, 0.0f};
    float sin_sum = 0.0f;
    float cos_sum = 0.0f;

    normalize_weights();

    for (int i = 0; i < EST_PARTICLE_COUNT; i++) {
        pose.x += particles[i].x * particles[i].weight;
        pose.y += particles[i].y * particles[i].weight;
        sin_sum += sinf(particles[i].theta) * particles[i].weight;
        cos_sum += cosf(particles[i].theta) * particles[i].weight;
    }

    pose.theta = atan2f(sin_sum, cos_sum);
    return pose;
}
