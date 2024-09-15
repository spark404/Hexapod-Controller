#include "dynamixel.h"
#include "protocol.h"
#include "api.h"
#include "crc.h"

#include <stddef.h>

dynamixel_result_t dynamixel_ll_ping(uint8_t identifier, dynamixel_bus_t *bus) {
	if (bus == NULL) {
		return DNM_API_ERR;
	}

	uint8_t packet[] = {
			0xFF, 0xFF, 0xFD, 0x00,
		    identifier, 0x03, 0x00, PING,
			0x00, 0x00
	};

	uint16_t crc = update_crc(0, packet, 8);
	packet[8] = crc & 0xFF;
	packet[9] = crc >> 8 & 0xFF;
	dynamixel_result_t r = bus->writeFunc(packet, 10, bus->pvContext);
	if (r != DNM_OK) {
		return r;
	}

	uint8_t rxBuffer[14];
	uint16_t n = 14;

	r = bus->readFunc(rxBuffer, n, bus->pvContext);
	if (r != DNM_OK) {
		return r;
	}

	crc = update_crc(0, rxBuffer, n - 2);
	int valid = rxBuffer[n-2] == (crc & 0xFF) && rxBuffer[n-1] == ((crc >> 8) & 0xFF);

	if (!valid) {
		return DNM_RECV_CRC_FAIL;
	}

	return DNM_OK;
}

dynamixel_result_t dynamixel_write(uint8_t identifier, uint16_t entry, uint8_t value, dynamixel_bus_t *bus) {
	if (bus == NULL) {
		return DNM_API_ERR;
	}

	size_t len = 13;
    uint8_t packet[] = {
			0xFF, 0xFF, 0xFD, 0x00,
		    identifier, len - 7, 0x00, WRITE,
			0x00, 0x00, 0x00, 0x00,
			0x00
	};

	// Write table number
	packet[8] = entry & 0xFF;
	packet[9] = (entry >> 8) & 0xFF;

	packet[10] = value & 0xFF;

	uint16_t crc = update_crc(0, packet, len-2);
	packet[len-2] = crc & 0xFF;
	packet[len-1] = crc >> 8 & 0xFF;

	dynamixel_result_t r = bus->writeFunc(packet, len, bus->pvContext);
	if (r != DNM_OK) {
		return r;
	}

	uint8_t rxBuffer[11];
	uint16_t n = 11;

	r = bus->readFunc(rxBuffer, n, bus->pvContext);
	if (r != DNM_OK) {
		return r;
	}

	if (n != 11) {
		return DNM_API_ERR;
	}

	crc = update_crc(0, rxBuffer, n - 2);
	int valid = rxBuffer[n-2] == (crc & 0xFF) && rxBuffer[n-1] == ((crc >> 8) & 0xFF);

	if (!valid) {
		return DNM_RECV_CRC_FAIL;
	}

	if (rxBuffer[7] != STATUS) {
		return DNM_RECV_NOT_STATUS;
	}

	if (rxBuffer[8] != 0) {
		return rxBuffer[8];
	}

	return DNM_OK;
}

dynamixel_result_t dynamixel_write2(uint8_t identifier, uint16_t entry, uint16_t value, dynamixel_bus_t *bus) {
	if (bus == NULL) {
		return DNM_API_ERR;
	}

	size_t len = 14;
    uint8_t packet[] = {
			0xFF, 0xFF, 0xFD, 0x00,
		    identifier, len - 7, 0x00, WRITE,
			0x00, 0x00, 0x00, 0x00,
			0x00, 0x00
	};

	// Write table number
	packet[8] = entry & 0xFF;
	packet[9] = (entry >> 8) & 0xFF;

	packet[10] = value & 0xFF;
	packet[11] = (value >> 8) && 0xFF;

	uint16_t crc = update_crc(0, packet, len-2);
	packet[len-2] = crc & 0xFF;
	packet[len-1] = crc >> 8 & 0xFF;

	dynamixel_result_t r = bus->writeFunc(packet, len, bus->pvContext);
	if (r != DNM_OK) {
		return r;
	}

	uint8_t rxBuffer[11];
	uint16_t n = 11;

	r = bus->readFunc(rxBuffer, n, bus->pvContext);
	if (r != DNM_OK) {
		return r;
	}

	if (n != 11) {
		return DNM_API_ERR;
	}

	crc = update_crc(0, rxBuffer, n - 2);
	int valid = rxBuffer[n-2] == (crc & 0xFF) && rxBuffer[n-1] == ((crc >> 8) & 0xFF);

	if (!valid) {
		return DNM_RECV_CRC_FAIL;
	}

	if (rxBuffer[7] != STATUS) {
		return DNM_RECV_NOT_STATUS;
	}

	if (rxBuffer[8] != 0) {
		return rxBuffer[8];
	}

	return DNM_OK;
}

dynamixel_result_t dynamixel_write4(uint8_t identifier, uint16_t entry, uint32_t value, dynamixel_bus_t *bus) {
	if (bus == NULL) {
		return DNM_API_ERR;
	}

	size_t len = 16;
    uint8_t packet[] = {
			0xFF, 0xFF, 0xFD, 0x00,
		    identifier, len - 7, 0x00, WRITE,
			0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00
	};

	// Write table number
	packet[8] = entry & 0xFF;
	packet[9] = (entry >> 8) & 0xFF;

	packet[10] = value & 0xFF;
	packet[11] = (value >> 8) & 0xFF;
	packet[12] = (value >> 16) & 0xFF;
	packet[13] = (value >> 24) & 0xFF;

	uint16_t crc = update_crc(0, packet, len-2);
	packet[len-2] = crc & 0xFF;
	packet[len-1] = crc >> 8 & 0xFF;

	dynamixel_result_t r = bus->writeFunc(packet, len, bus->pvContext);
	if (r != DNM_OK) {
		return r;
	}

	uint8_t rxBuffer[11];
	uint16_t n = 11;

	r = bus->readFunc(rxBuffer, n, bus->pvContext);
	if (r != DNM_OK) {
		return r;
	}

	if (n != 11) {
		return DNM_API_ERR;
	}

	crc = update_crc(0, rxBuffer, n - 2);
	int valid = rxBuffer[n-2] == (crc & 0xFF) && rxBuffer[n-1] == ((crc >> 8) & 0xFF);

	if (!valid) {
		return DNM_RECV_CRC_FAIL;
	}

	if (rxBuffer[7] != STATUS) {
		return DNM_RECV_NOT_STATUS;
	}

	if (rxBuffer[8] != 0) {
		return rxBuffer[8];
	}

	return DNM_OK;
}

dynamixel_result_t dynamixel_read(uint8_t identifier, uint16_t entry, uint8_t entry_size, uint32_t *value, dynamixel_bus_t *bus) {
	if (bus == NULL) {
		return DNM_API_ERR;
	}

	size_t len = 14;
    uint8_t packet[] = {
			0xFF, 0xFF, 0xFD, 0x00,
		    identifier, len - 7, 0x00, READ,
			0x00, 0x00, 0x00, 0x00,
			0x00, 0x00
	};

	if (entry_size != 1 && entry_size != 2 && entry_size != 4) {
		return DNM_API_ERR;
	}

	// Write table number
	packet[8] = entry & 0xFF;
	packet[9] = (entry >> 8) & 0xFF;

	packet[10] = entry_size & 0xFF;
	packet[11] = (entry_size >> 8) & 0xFF;

	uint16_t crc = update_crc(0, packet, len-2);
	packet[len-2] = crc & 0xFF;
	packet[len-1] = crc >> 8 & 0xFF;

	dynamixel_result_t r = bus->writeFunc(packet, len, bus->pvContext);
	if (r != DNM_OK) {
		return r;
	}

	uint8_t rxBuffer[11];
	uint16_t n = 11;

	r = bus->readFunc(rxBuffer, n, bus->pvContext);
	if (r != DNM_OK) {
		return r;
	}

	if (n < 11) {
		return DNM_API_ERR;
	}

	crc = update_crc(0, rxBuffer, n - 2);
	int valid = rxBuffer[n-2] == (crc & 0xFF) && rxBuffer[n-1] == ((crc >> 8) & 0xFF);

	if (!valid) {
		return DNM_RECV_CRC_FAIL;
	}

	if (rxBuffer[7] != STATUS) {
		return DNM_RECV_NOT_STATUS;
	}

	if (rxBuffer[8] != 0) {
		return rxBuffer[8];
	}

	if (n == 12) {
		*value = rxBuffer[9];
	} else if (n == 13) {
		*value = rxBuffer[9] | rxBuffer[10] << 8;
	} else {
		*value = rxBuffer[9] | rxBuffer[10] << 8 | rxBuffer[11] << 16 | rxBuffer[12] << 24;
	}

	return DNM_OK;
}

