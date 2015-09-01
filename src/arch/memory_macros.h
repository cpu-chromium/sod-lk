#ifndef ARCH_MEMORY_MACROS_H
#define ARCH_MEMORY_MACROS_H

// Use the following macros to decorate memory mapped registers.

#define A_I   volatile const       //  Read-only register.
#define A_O   volatile             //  Write-only register.
#define A_IO  volatile             //  Read-Write register.

#define ASM_DSB __asm volatile ("dsb 0xF":::"memory")
#define ASM_ISB __asm volatile ("isb 0xF":::"memory")

#endif // ARCH_MEMORY_MACROS_H
