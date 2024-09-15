#ifndef DYNAMIXEL_CRC_H
#define DYNAMIXEL_CRC_H

#include <stdint.h>

uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);

#endif /* DYNAMIXEL_CRC_H */
