#include "arch/arm/cortex-m/system_control_block.h"

#include "arch/memory_macros.h"


#define SCB_CCR_IC_Pos 17
#define SCB_CCR_IC_Msk (1UL << SCB_CCR_IC_Pos)

#define SCB_CCSIDR_NUMSETS_Pos 13
#define SCB_CCSIDR_NUMSETS_Msk (0x7FFFUL << SCB_CCSIDR_NUMSETS_Pos)

#define SCB_CCSIDR_ASSOCIATIVITY_Pos 3
#define SCB_CCSIDR_ASSOCIATIVITY_Msk (0x3FFUL << SCB_CCSIDR_ASSOCIATIVITY_Pos)

#define SCB_CCSIDR_LINESIZE_Pos 0
#define SCB_CCSIDR_LINESIZE_Msk (7UL /*shift r 0*/)

#define SCB_CCR_DC_Pos 16  // Cache enable bit Position.
#define SCB_CCR_DC_Msk (1UL << SCB_CCR_DC_Pos)   // Cache enable bit Mask

// Cache Size ID Register Macros
#define CCSIDR_WAYS(x) (((x) & SCB_CCSIDR_ASSOCIATIVITY_Msk) >> SCB_CCSIDR_ASSOCIATIVITY_Pos)
#define CCSIDR_SETS(x) (((x) & SCB_CCSIDR_NUMSETS_Msk) >> SCB_CCSIDR_NUMSETS_Pos      )
#define CCSIDR_LSSHIFT(x) (((x) & SCB_CCSIDR_LINESIZE_Msk) /*shift r 0*/)

// SCB Application Interrupt and Reset Control Register Definitions
#define SCB_AIRCR_VECTKEY_Pos 16            
#define SCB_AIRCR_VECTKEY_Msk (0xFFFFUL << SCB_AIRCR_VECTKEY_Pos)

#define SCB_AIRCR_PRIGROUP_Pos 8
#define SCB_AIRCR_PRIGROUP_Msk (7UL << SCB_AIRCR_PRIGROUP_Pos)


void EnableICache() {
  ASM_DSB;
  ASM_ISB;
  SYSCTRLBLK->ICIALLU = 0UL;                     // invalidate I-Cache
  SYSCTRLBLK->CCR |=  (uint32_t)SCB_CCR_IC_Msk;  // enable I-Cache
  ASM_DSB;
  ASM_ISB;
}

void EnableDCache() {
  SYSCTRLBLK->CSSELR = (0UL << 1) | 0UL;         // Level 1 data cache
  uint32_t ccsidr  = SYSCTRLBLK->CCSIDR;
  auto sets    = (uint32_t)(CCSIDR_SETS(ccsidr));
  auto sshift  = (uint32_t)(CCSIDR_LSSHIFT(ccsidr) + 4UL);
  auto ways    = (uint32_t)(CCSIDR_WAYS(ccsidr));
  auto wshift  = (uint32_t)((uint32_t)__builtin_clz(ways) & 0x1FUL);

  ASM_DSB;

  do {                                   // invalidate D-Cache
    auto tmpways = ways;
    do {
      auto sw = ((tmpways << wshift) | (sets << sshift));
      SYSCTRLBLK->DCISW = sw;
    } while(tmpways--);
  } while(sets--);

  ASM_DSB;
  SYSCTRLBLK->CCR |=  (uint32_t)SCB_CCR_DC_Msk;   // enable D-Cache
  ASM_DSB;
  ASM_ISB;  
}

//  The function sets the priority grouping field using the required unlock sequence.
//    The parameter PriorityGroup is assigned to the field SCB->AIRCR [10:8].
//    Only values from 0..7 are used.
//    In case of a conflict between priority grouping and available
//    priority bits (__NVIC_PRIO_BITS), the smallest possible priority group is set.

void SetPriorityGrouping(uint32_t PriorityGroup) {
  // Only values 0..7 are used.
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);      
  uint32_t reg_value  =  SYSCTRLBLK->AIRCR;
  reg_value &= ~((uint32_t)(SCB_AIRCR_VECTKEY_Msk | SCB_AIRCR_PRIGROUP_Msk));
  // Insert write key and priorty group.
  reg_value = (reg_value |
              ((uint32_t)0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
              (PriorityGroupTmp << 8));
  SYSCTRLBLK->AIRCR = reg_value;
}
