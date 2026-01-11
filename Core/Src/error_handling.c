#include "error_handling.h"

#include <stdio.h>
#include "cmsis_gcc.h"
#include "stm32f401xe.h"

void error_print_backtrace_from_sp(uint32_t *sp)
{
    printf("\nBacktrace (best effort):\n");

    for (int i = 0; i < 16; i++) {
        uint32_t addr = sp[i];

        if ((addr >= FLASH_BASE) &&
            (addr < FLASH_END) &&
            (addr & 1)) {

            printf("  #%02d 0x%08lx\n", i, addr & ~1UL);
            }
    }
}

void error_print_backtrace(void)
{
    uint32_t *sp;
    __asm volatile ("mrs %0, psp" : "=r"(sp));
    error_print_backtrace_from_sp(sp);
}

void hardfault_c(uint32_t *sp)
{
    __disable_irq();

    uint32_t r0  = sp[0];
    uint32_t r1  = sp[1];
    uint32_t r2  = sp[2];
    uint32_t r3  = sp[3];
    uint32_t r12 = sp[4];
    uint32_t lr  = sp[5];
    uint32_t pc  = sp[6];
    uint32_t psr = sp[7];

    printf("\n\n=== HARD FAULT ===\n");
    printf("R0  = 0x%08lx\n", r0);
    printf("R1  = 0x%08lx\n", r1);
    printf("R2  = 0x%08lx\n", r2);
    printf("R3  = 0x%08lx\n", r3);
    printf("R12 = 0x%08lx\n", r12);
    printf("LR  = 0x%08lx\n", lr);
    printf("PC  = 0x%08lx\n", pc);
    printf("PSR = 0x%08lx\n", psr);

    error_print_backtrace_from_sp(sp);

    printf("System halted.\n");

    for (;;) {
        __BKPT(0);
    }
}


