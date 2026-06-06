#include "serial_parser.h"

#include <stdlib.h>
#include <string.h>

static bool parse_float_after(const char *base, const char *key, float *out) {
    const char *p = strstr(base, key);
    if (!p) {
        return false;
    }
    p += strlen(key);
    *out = strtof(p, NULL);
    return true;
}

static bool parse_int_after(const char *base, const char *key, int32_t *out) {
    const char *p = strstr(base, key);
    if (!p) {
        return false;
    }
    p += strlen(key);
    *out = (int32_t)strtol(p, NULL, 10);
    return true;
}

static const char *section_after(const char *line, const char *key) {
    const char *p = strstr(line, key);
    if (!p) {
        return NULL;
    }
    return p + strlen(key);
}

static void parse_anchor_id(const char *obj, char *id, int id_len) {
    const char *p = strstr(obj, "\"id\":\"");
    int i = 0;

    if (!p || id_len <= 0) {
        return;
    }

    p += 6;
    while (*p && *p != '"' && i < id_len - 1) {
        id[i++] = *p++;
    }
    id[i] = '\0';
}

static void parse_uwb_array(const char *line, SensorPacket *packet) {
    const char *p = strstr(line, "\"uwb\":[");
    packet->uwb_count = 0;

    if (!p) {
        return;
    }

    p += 7;
    while ((p = strstr(p, "\"id\":\"")) != NULL && packet->uwb_count < SENSOR_MAX_UWB) {
        UwbMeasurement *m = &packet->uwb[packet->uwb_count];
        memset(m, 0, sizeof(*m));

        parse_anchor_id(p, m->id, SENSOR_ANCHOR_ID_LEN);
        parse_float_after(p, "\"x\":", &m->x);
        parse_float_after(p, "\"y\":", &m->y);
        parse_float_after(p, "\"z\":", &m->z);
        parse_float_after(p, "\"distance\":", &m->distance);
        parse_float_after(p, "\"std\":", &m->std);

        packet->uwb_count++;
        p++;
    }
}

bool parse_sensor_packet(const char *line, SensorPacket *packet) {
    const char *state;
    const char *imu;
    const char *ticks;

    if (!line || !packet) {
        return false;
    }

    memset(packet, 0, sizeof(*packet));

    if (!parse_int_after(line, "\"seq\":", &packet->seq)) {
        return false;
    }

    state = section_after(line, "\"state\":{");
    if (state) {
        packet->has_truth = true;
        parse_float_after(state, "\"x\":", &packet->truth_x);
        parse_float_after(state, "\"y\":", &packet->truth_y);
        parse_float_after(state, "\"theta\":", &packet->truth_theta);
    }

    imu = section_after(line, "\"imu\":{");
    if (!imu) {
        return false;
    }
    if (!parse_float_after(imu, "\"acc_x\":", &packet->acc_x)) {
        return false;
    }
    parse_float_after(imu, "\"acc_y\":", &packet->acc_y);
    parse_float_after(imu, "\"acc_z\":", &packet->acc_z);
    parse_float_after(imu, "\"gyro_x\":", &packet->gyro_x);
    parse_float_after(imu, "\"gyro_y\":", &packet->gyro_y);
    if (!parse_float_after(imu, "\"gyro_z\":", &packet->gyro_z)) {
        return false;
    }

    ticks = section_after(line, "\"hall_ticks\":{");
    if (!ticks) {
        return false;
    }
    parse_int_after(ticks, "\"front_left\":", &packet->front_left_ticks);
    parse_int_after(ticks, "\"front_right\":", &packet->front_right_ticks);
    parse_int_after(ticks, "\"rear_left\":", &packet->rear_left_ticks);
    parse_int_after(ticks, "\"rear_right\":", &packet->rear_right_ticks);

    parse_uwb_array(line, packet);
    return true;
}
