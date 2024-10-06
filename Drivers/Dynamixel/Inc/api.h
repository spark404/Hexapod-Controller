#ifndef DYNAMIXEL_API_H
#define DYNAMIXEL_API_H

#include <stdint.h>
#include "dynamixel.h"

dynamixel_result_t dynamixel_ll_ping(uint8_t identifier, dynamixel_bus_t bus);

dynamixel_result_t dynamixel_write(uint8_t identifier, uint16_t entry, uint8_t value, dynamixel_bus_t bus);
dynamixel_result_t dynamixel_write2(uint8_t identifier, uint16_t entry, uint16_t value, dynamixel_bus_t bus);
dynamixel_result_t dynamixel_write4(uint8_t identifier, uint16_t entry, uint32_t value, dynamixel_bus_t bus);

dynamixel_result_t dynamixel_read(uint8_t identifier, uint16_t entry, uint8_t entry_size, uint32_t *value, dynamixel_bus_t bus);

#endif /* DYNAMIXEL_CRC_H */
