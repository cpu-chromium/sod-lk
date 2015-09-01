#ifndef ARCH_ARM_CORTEX_M_NESTED_INTERRUPT_CONTROLLER_H
#define ARCH_ARM_CORTEX_M_NESTED_INTERRUPT_CONTROLLER_H

#include <stdint.h>
#include "arch/memory_macros.h"
#include "arch/arm/cortex-m/memory_map.h"
#include "arch/arm/cortex-m/interrupt_types.h"

// Nested Vectored Interrupt Controller (NVIC)
struct NestedVectInterruptController {
  A_IO uint32_t ISER[8];       // 0x000 Interrupt Set Enable Register
       uint32_t RESERVED0[24];
  A_IO uint32_t ICER[8];       // 0x080 Interrupt Clear Enable Register
       uint32_t RSERVED1[24];
  A_IO uint32_t ISPR[8];       // 0x100 Interrupt Set Pending Register
       uint32_t RESERVED2[24];
  A_IO uint32_t ICPR[8];       // 0x180 Interrupt Clear Pending Register
       uint32_t RESERVED3[24];
  A_IO uint32_t IABR[8];       // 0x200 Interrupt Active bit Register
       uint32_t RESERVED4[56];
  A_IO uint8_t  IP[240];       // 0x300 Interrupt Priority Register (8Bit wide)
       uint32_t RESERVED5[644];
  A_O  uint32_t STIR;          // 0xE00 Software Trigger Interrupt Register
};

void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority);

#define NVIC_PRIO_BITS 3

#define NVECINTCTRLR ((NestedVectInterruptController*) (SCS_BASE +  0x0100UL))

#endif  // ARCH_ARM_CORTEX_M_NESTED_INTERRUPT_CONTROLLER_H
