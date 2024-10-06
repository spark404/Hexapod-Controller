//
// Created by Hugo Trippaers on 06/10/2024.
//
#include "dynamixel.h"
#include "dynamixel_internal.h"
#include "api.h"
#include "protocol.h"

#include <stddef.h>

dynamixel_result_t dynamixel_bus_init(dynamixel_bus_t bus, dynamixelReadFunc_t readFunc, dynamixelWriteFunc_t writeFunc, void *pvContext) {
    bus->readFunc = readFunc;
    bus->writeFunc = writeFunc;
    bus->pvContext = pvContext;

    return DNM_OK;
}