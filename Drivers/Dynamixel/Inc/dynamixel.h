#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include <stdint.h>
#include <stddef.h>

/* Supported dynamixel types */
#define DYNAMIXEL_XL430 0x01

/* Result type for all public functions */
typedef uint8_t dynamixel_result_t;

#define DNM_OK               0
#define DNM_API_ERR         64
#define DNM_RECV_CRC_FAIL   65
#define DNM_RECV_NOT_STATUS 66
#define DNM_NOT_SUPPORTED   67
#define DNM_LL_ERR          68
#define DNM_LL_TIMEOUT      69

typedef dynamixel_result_t (*dynamixelWriteFunc_t) (uint8_t *txBuffer, size_t size, void *pvArgument);
typedef dynamixel_result_t (*dynamixelReadFunc_t) (uint8_t *rxBuffer, size_t size, void *pvArgument);

typedef struct {
	dynamixelWriteFunc_t writeFunc;
	dynamixelReadFunc_t readFunc;
	void *pvContext;
} dynamixel_bus_t;

typedef struct {
	uint8_t id;
	uint8_t type;
	uint8_t initialized;
	dynamixel_bus_t *bus;
} dynamixel_servo_t;

dynamixel_result_t dynamixel_ping(dynamixel_servo_t *servo);
dynamixel_result_t dynamixel_led_set(dynamixel_servo_t *servo);
dynamixel_result_t dynamixel_led_reset(dynamixel_servo_t *servo);

#endif /* DYNAMIXEL_H */
