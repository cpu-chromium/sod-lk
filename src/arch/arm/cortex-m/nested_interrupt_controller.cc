#include "arch/arm/cortex-m/nested_interrupt_controller.h"

#include "arch/arm/cortex-m/system_control_block.h"
// Set Interrupt Priority
//   The function sets the priority of an interrupt.
//   note: The priority cannot be set for every core interrupt.
//   |IRQn|  Interrupt number.
//   |priority|  Priority to set.

void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority) {
  if((int32_t)IRQn < 0) {
    SYSCTRLBLK->SHPR[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] =
        (uint8_t)((priority << (8 - NVIC_PRIO_BITS)) & (uint32_t)0xFFUL);
  } else {
    NVECINTCTRLR->IP[((uint32_t)(int32_t)IRQn)] =
        (uint8_t)((priority << (8 - NVIC_PRIO_BITS)) & (uint32_t)0xFFUL);
  }
}
