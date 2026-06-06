#ifndef SENSOR_PACKET_H
#define SENSOR_PACKET_H

#include <stdbool.h>
#include <stdint.h>

#define SENSOR_MAX_UWB 5
#define SENSOR_ANCHOR_ID_LEN 8

typedef struct {
    char id[SENSOR_ANCHOR_ID_LEN];
    float x;
    float y;
    float z;
    float distance;
    float std;
} UwbMeasurement;

typedef struct {
    int32_t seq;

    bool has_truth;
    float truth_x;
    float truth_y;
    float truth_theta;

    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;

    int32_t front_left_ticks;
    int32_t front_right_ticks;
    int32_t rear_left_ticks;
    int32_t rear_right_ticks;

    int uwb_count;
    UwbMeasurement uwb[SENSOR_MAX_UWB];
} SensorPacket;

#endif
