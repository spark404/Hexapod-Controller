//
// Created by Hugo Trippaers on 11/01/2026.
//

#ifndef ERROR_HANDLING_H
#define ERROR_HANDLING_H

#include <stdint.h>

#define FLASH_START 0x08000000UL
// Already defined
// #define FLASH_END   0x08080000UL  // adjust to your STM32F401 size

void error_print_backtrace_from_sp(uint32_t *sp);
void error_print_backtrace(void);
void hardfault_c(uint32_t *sp);

#endif //ERROR_HANDLING_H
