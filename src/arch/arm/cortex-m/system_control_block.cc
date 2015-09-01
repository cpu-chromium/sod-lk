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
