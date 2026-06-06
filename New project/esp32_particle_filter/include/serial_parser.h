#ifndef SERIAL_PARSER_H
#define SERIAL_PARSER_H

#include <stdbool.h>
#include "sensor_packet.h"

bool parse_sensor_packet(const char *line, SensorPacket *packet);

#endif
