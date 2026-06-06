#include <Arduino.h>

extern "C" {
#include "estimator_config.h"
#include "mecanum.h"
#include "particle_filter.h"
#include "sensor_packet.h"
#include "serial_parser.h"
}

static char line_buffer[EST_LINE_BUFFER_SIZE];
static size_t line_len = 0;
static bool filter_ready = false;

static void print_estimate(const SensorPacket *packet, const Pose2D *pose, float neff) {
    Serial.print("{\"seq\":");
    Serial.print(packet->seq);
    Serial.print(",\"estimate\":{\"x\":");
    Serial.print(pose->x, 4);
    Serial.print(",\"y\":");
    Serial.print(pose->y, 4);
    Serial.print(",\"theta\":");
    Serial.print(pose->theta, 5);
    Serial.print("},\"neff\":");
    Serial.print(neff, 2);

    if (packet->has_truth) {
        Serial.print(",\"truth\":{\"x\":");
        Serial.print(packet->truth_x, 4);
        Serial.print(",\"y\":");
        Serial.print(packet->truth_y, 4);
        Serial.print(",\"theta\":");
        Serial.print(packet->truth_theta, 5);
        Serial.print("},\"error\":{\"x\":");
        Serial.print(pose->x - packet->truth_x, 4);
        Serial.print(",\"y\":");
        Serial.print(pose->y - packet->truth_y, 4);
        Serial.print(",\"theta\":");
        Serial.print(pose->theta - packet->truth_theta, 5);
        Serial.print("}");
    }

    Serial.println("}");
}

static void handle_line(const char *line) {
    SensorPacket packet;

    if (!parse_sensor_packet(line, &packet)) {
        Serial.println("{\"error\":\"parse_failed\"}");
        return;
    }

    if (!filter_ready) {
        pf_init(EST_INITIAL_X_M, EST_INITIAL_Y_M, EST_INITIAL_THETA_RAD);
        filter_ready = true;
    }

    const OdometryDelta odom = mecanum_delta_from_packet(&packet);
    pf_predict(&odom);
    pf_update_uwb(&packet);
    pf_resample_if_needed();

    const Pose2D pose = pf_estimate();
    print_estimate(&packet, &pose, pf_effective_sample_size());
}

void setup() {
    Serial.begin(EST_SERIAL_BAUD);
    delay(500);
    Serial.println("{\"status\":\"particle_filter_ready\"}");
}

void loop() {
    while (Serial.available() > 0) {
        const char c = (char)Serial.read();

        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            line_buffer[line_len] = '\0';
            if (line_len > 0) {
                handle_line(line_buffer);
            }
            line_len = 0;
            continue;
        }

        if (line_len < EST_LINE_BUFFER_SIZE - 1) {
            line_buffer[line_len++] = c;
        } else {
            line_len = 0;
            Serial.println("{\"error\":\"line_too_long\"}");
        }
    }
}
