#include "dynamixel.h"
#include "dynamixel_internal.h"
#include "api.h"
#include "protocol.h"

#include <stddef.h>


dynamixel_result_t dynamixel_init(dynamixel_servo_t servo, uint8_t id, uint8_t type, dynamixel_bus_t bus)
{
	servo->id = id;
	servo->type = type;
	servo->initialized = 1;
	servo->bus = bus;

    return DNM_OK;
}

dynamixel_result_t dynamixel_ping(dynamixel_servo_t servo) {
	return dynamixel_ll_ping(servo->id, servo->bus);
}

dynamixel_result_t dynamixel_led_set(dynamixel_servo_t servo) {
	if (servo == NULL) {
		return DNM_API_ERR;
	}

	if (servo->type != DYNAMIXEL_XL430) {
		return DNM_NOT_SUPPORTED;
	}

	return dynamixel_write(servo->id, XL430_CT_RAM_LED, 1, servo->bus);
}

dynamixel_result_t dynamixel_led_reset(dynamixel_servo_t servo) {
	if (servo == NULL) {
		return DNM_API_ERR;
	}

	if (servo->type != DYNAMIXEL_XL430) {
		return DNM_NOT_SUPPORTED;
	}

	return dynamixel_write(servo->id, XL430_CT_RAM_LED, 0, servo->bus);
}
