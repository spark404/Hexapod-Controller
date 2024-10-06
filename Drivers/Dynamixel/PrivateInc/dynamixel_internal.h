struct dynamixel_bus {
	dynamixelWriteFunc_t writeFunc;
	dynamixelReadFunc_t readFunc;
	void *pvContext;
};

struct dynamixel_servo {
	uint8_t id;
	uint8_t type;
	uint8_t initialized;
	dynamixel_bus_t bus;
};
