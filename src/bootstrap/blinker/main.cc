#include <stdint.h>
#include "interrupt_types.h"

#define __I     volatile const       // Defines 'read only' permissions     
#define __O     volatile             // Defines 'write only' permissions    
#define __IO    volatile             // Defines 'read / write' permissions  

// System Control Block.
struct SCB_Type {
  __I  uint32_t CPUID;         // 0x000 (R/ )  CPUID Base Register                       
  __IO uint32_t ICSR;          // 0x004 (R/W)  Interrupt Control and State Register      
  __IO uint32_t VTOR;          // 0x008 (R/W)  Vector Table Offset Register              
  __IO uint32_t AIRCR;         // 0x00C (R/W)  Application Interrupt and Reset Control Register
  __IO uint32_t SCR;           // 0x010 (R/W)  System Control Register                   
  __IO uint32_t CCR;           // 0x014 (R/W)  Configuration Control Register            
  __IO uint8_t  SHPR[12];      // 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15)
  __IO uint32_t SHCSR;         // 0x024 (R/W)  System Handler Control and State Register 
  __IO uint32_t CFSR;          // 0x028 (R/W)  Configurable Fault Status Register        
  __IO uint32_t HFSR;          // 0x02C (R/W)  HardFault Status Register                 
  __IO uint32_t DFSR;          // 0x030 (R/W)  Debug Fault Status Register               
  __IO uint32_t MMFAR;         // 0x034 (R/W)  MemManage Fault Address Register          
  __IO uint32_t BFAR;          // 0x038 (R/W)  BusFault Address Register                 
  __IO uint32_t AFSR;          // 0x03C (R/W)  Auxiliary Fault Status Register           
  __I  uint32_t ID_PFR[2];     // 0x040 (R/ )  Processor Feature Register                
  __I  uint32_t ID_DFR;        // 0x048 (R/ )  Debug Feature Register                    
  __I  uint32_t ID_AFR;        // 0x04C (R/ )  Auxiliary Feature Register                
  __I  uint32_t ID_MFR[4];     // 0x050 (R/ )  Memory Model Feature Register             
  __I  uint32_t ID_ISAR[5];    // 0x060 (R/ )  Instruction Set Attributes Register       
       uint32_t RESERVED0[1];
  __I  uint32_t CLIDR;         // 0x078 (R/ )  Cache Level ID register                   
  __I  uint32_t CTR;           // 0x07C (R/ )  Cache Type register                       
  __I  uint32_t CCSIDR;        // 0x080 (R/ )  Cache Size ID Register                    
  __IO uint32_t CSSELR;        // 0x084 (R/W)  Cache Size Selection Register             
  __IO uint32_t CPACR;         // 0x088 (R/W)  Coprocessor Access Control Register       
       uint32_t RESERVED3[93];
  __O  uint32_t STIR;          // 0x200 ( /W)  Software Triggered Interrupt Register     
       uint32_t RESERVED4[15];
  __I  uint32_t MVFR0;         // 0x240 (R/ )  Media and VFP Feature Register 0          
  __I  uint32_t MVFR1;         // 0x244 (R/ )  Media and VFP Feature Register 1          
  __I  uint32_t MVFR2;         // 0x248 (R/ )  Media and VFP Feature Register 1          
       uint32_t RESERVED5[1];
  __O  uint32_t ICIALLU;       // 0x250 ( /W)  I-Cache Invalidate All to PoU             
       uint32_t RESERVED6[1];
  __O  uint32_t ICIMVAU;       // 0x258 ( /W)  I-Cache Invalidate by MVA to PoU          
  __O  uint32_t DCIMVAC;       // 0x25C ( /W)  D-Cache Invalidate by MVA to PoC          
  __O  uint32_t DCISW;         // 0x260 ( /W)  D-Cache Invalidate by Set-way             
  __O  uint32_t DCCMVAU;       // 0x264 ( /W)  D-Cache Clean by MVA to PoU               
  __O  uint32_t DCCMVAC;       // 0x268 ( /W)  D-Cache Clean by MVA to PoC               
  __O  uint32_t DCCSW;         // 0x26C ( /W)  D-Cache Clean by Set-way                  
  __O  uint32_t DCCIMVAC;      // 0x270 ( /W)  D-Cache Clean and Invalidate by MVA to PoC
  __O  uint32_t DCCISW;        // 0x274 ( /W)  D-Cache Clean and Invalidate by Set-way   
       uint32_t RESERVED7[6];
  __IO uint32_t ITCMCR;        // 0x290 (R/W)  Instruction Tightly-Coupled Memory Control Register
  __IO uint32_t DTCMCR;        // 0x294 (R/W)  Data Tightly-Coupled Memory Control Registers
  __IO uint32_t AHBPCR;        // 0x298 (R/W)  AHBP Control Register                     
  __IO uint32_t CACR;          // 0x29C (R/W)  L1 Cache Control Register                 
  __IO uint32_t AHBSCR;        // 0x2A0 (R/W)  AHB Slave Control Register                
       uint32_t RESERVED8[1];
  __IO uint32_t ABFSR;         // 0x2A8 (R/W)  Auxiliary Bus Fault Status Register       
};

// Nested Vectored Interrupt Controller (NVIC)
struct NVIC_Type {
  __IO uint32_t ISER[8];          // 0x000 (R/W)  Interrupt Set Enable Register
       uint32_t RESERVED0[24];
  __IO uint32_t ICER[8];          // 0x080 (R/W)  Interrupt Clear Enable Register
       uint32_t RSERVED1[24];
  __IO uint32_t ISPR[8];          // 0x100 (R/W)  Interrupt Set Pending Register
       uint32_t RESERVED2[24];
  __IO uint32_t ICPR[8];          // 0x180 (R/W)  Interrupt Clear Pending Register
       uint32_t RESERVED3[24];
  __IO uint32_t IABR[8];          // 0x200 (R/W)  Interrupt Active bit Register
       uint32_t RESERVED4[56];
  __IO uint8_t  IP[240];          // 0x300 (R/W)  Interrupt Priority Register (8Bit wide)
       uint32_t RESERVED5[644];
  __O  uint32_t STIR;             // 0xE00 ( /W)  Software Trigger Interrupt Register
};

// System Timer (SysTick).
struct SysTick_Type {
  __IO uint32_t CTRL;             // 0x000 (R/W)  SysTick Control and Status Register
  __IO uint32_t LOAD;             // 0x004 (R/W)  SysTick Reload Value Register
  __IO uint32_t VAL;              // 0x008 (R/W)  SysTick Current Value Register
  __I  uint32_t CALIB;            // 0x00C (R/ )  SysTick Calibration Register
};

#define SCS_BASE (0xE000E000UL)    // System Control Space Base Address.
#define SysTick_BASE (SCS_BASE +  0x0010UL)
#define NVIC_BASE (SCS_BASE +  0x0100UL)
#define SCB_BASE (SCS_BASE +  0x0D00UL)

#define SCB ((SCB_Type*) SCB_BASE)
#define SysTick ((SysTick_Type *) SysTick_BASE) 
#define NVIC ((NVIC_Type*) NVIC_BASE)

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

// Priority groups.
#define NVIC_PRIORITYGROUP_0 ((uint32_t)0x00000007) // 0 bits for pre-emption priority
                                                    // 4 bits for subpriority
#define NVIC_PRIORITYGROUP_1 ((uint32_t)0x00000006) // 1 bits for pre-emption priority
                                                    // 3 bits for subpriority
#define NVIC_PRIORITYGROUP_2 ((uint32_t)0x00000005) // 2 bits for pre-emption priority
                                                    // 2 bits for subpriority
#define NVIC_PRIORITYGROUP_3 ((uint32_t)0x00000004) // 3 bits for pre-emption priority
                                                    // 1 bits for subpriority
#define NVIC_PRIORITYGROUP_4 ((uint32_t)0x00000003) // 4 bits for pre-emption priority
                                                    // 0 bits for subpriority
#define __NVIC_PRIO_BITS 3

//* SysTick Reload Register Definitions
#define SysTick_CTRL_CLKSOURCE_Pos 2                                    
#define SysTick_CTRL_CLKSOURCE_Msk (1UL << SysTick_CTRL_CLKSOURCE_Pos)

#define SysTick_CTRL_TICKINT_Pos 1
#define SysTick_CTRL_TICKINT_Msk (1UL << SysTick_CTRL_TICKINT_Pos)

#define SysTick_CTRL_ENABLE_Pos 0
#define SysTick_CTRL_ENABLE_Msk (1UL /*l shift 0*/)

#define SysTick_LOAD_RELOAD_Pos 0   
#define SysTick_LOAD_RELOAD_Msk (0xFFFFFFUL /*l shift 0*/)

#define  TICK_INT_PRIORITY ((uint32_t)0x0F)

// Count leading zeros
#define __CLZ __builtin_clz

__attribute__((always_inline)) void __DSB() {
  __asm volatile ("dsb 0xF":::"memory");
}

__attribute__((always_inline)) void __ISB() {
  __asm volatile ("isb 0xF":::"memory");
}

__attribute__((always_inline)) void __WFI() {
  __asm volatile ("wfi");
}

inline void EnableICache() {
  __DSB();
  __ISB();
  SCB->ICIALLU = 0UL;                     // invalidate I-Cache
  SCB->CCR |=  (uint32_t)SCB_CCR_IC_Msk;  // enable I-Cache
  __DSB();
  __ISB();
}

inline void EnableDCache() {
  SCB->CSSELR = (0UL << 1) | 0UL;         // Level 1 data cache
  uint32_t ccsidr  = SCB->CCSIDR;
  auto sets    = (uint32_t)(CCSIDR_SETS(ccsidr));
  auto sshift  = (uint32_t)(CCSIDR_LSSHIFT(ccsidr) + 4UL);
  auto ways    = (uint32_t)(CCSIDR_WAYS(ccsidr));
  auto wshift  = (uint32_t)((uint32_t)__CLZ(ways) & 0x1FUL);

  __DSB();

  do {                                   // invalidate D-Cache
    auto tmpways = ways;
    do {
      auto sw = ((tmpways << wshift) | (sets << sshift));
      SCB->DCISW = sw;
    } while(tmpways--);
  } while(sets--);

  __DSB();
  SCB->CCR |=  (uint32_t)SCB_CCR_DC_Msk;   // enable D-Cache
  __DSB();
  __ISB();  
}

#define __BKPT(value) __asm volatile ("bkpt "#value)

void halt_bp() {
  __BKPT(6);  
}

//  The function sets the priority grouping field using the required unlock sequence.
//    The parameter PriorityGroup is assigned to the field SCB->AIRCR [10:8].
//    Only values from 0..7 are used.
//    In case of a conflict between priority grouping and available
//    priority bits (__NVIC_PRIO_BITS), the smallest possible priority group is set.

inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup) {
  // Only values 0..7 are used.
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);      
  uint32_t reg_value  =  SCB->AIRCR;
  reg_value &= ~((uint32_t)(SCB_AIRCR_VECTKEY_Msk | SCB_AIRCR_PRIGROUP_Msk));
  // Insert write key and priorty group.
  reg_value = (reg_value |
              ((uint32_t)0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
              (PriorityGroupTmp << 8));
  SCB->AIRCR = reg_value;
}

// Set Interrupt Priority
//   The function sets the priority of an interrupt.
//   note: The priority cannot be set for every core interrupt.
//   |IRQn|  Interrupt number.
//   |priority|  Priority to set.

inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority) {
  if((int32_t)IRQn < 0) {
    SCB->SHPR[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] =
        (uint8_t)((priority << (8 - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL);
  } else {
    NVIC->IP[((uint32_t)(int32_t)IRQn)] =
        (uint8_t)((priority << (8 - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL);
  }
}

// System Tick Configuration
//   The function initializes the System Timer and its interrupt, and starts the System Tick Timer.
//   Counter is in free running mode to generate periodic interrupts.
//   |ticks|  Number of ticks between two interrupts.

inline bool SysTick_Config(uint32_t ticks) {
  if ((ticks - 1UL) > SysTick_LOAD_RELOAD_Msk)
    return false;    // Reload value impossible.

  SysTick->LOAD  = (uint32_t)(ticks - 1UL);
  NVIC_SetPriority(SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
  SysTick->VAL   = 0UL;
  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                   SysTick_CTRL_TICKINT_Msk   |
                   SysTick_CTRL_ENABLE_Msk;
  return true;
}

inline bool InitSysTick(uint32_t ticks) {
  if (!SysTick_Config(ticks))
    return false;
  NVIC_SetPriority(SysTick_IRQn, TICK_INT_PRIORITY);
  return true;
}

volatile uint32_t milisecs_count = 0;

extern "C" void SysTick_Handler() {
  ++milisecs_count;
}

void DelayMS(uint32_t wait_ms) {
  auto start_ms = milisecs_count;
  while ((milisecs_count - start_ms) < wait_ms) {
    __WFI();
  }
}

int main() {
  EnableICache();
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  if (!InitSysTick(16000))
    halt_bp();
  while (true) {
    DelayMS(500);
  }
}

extern "C" void __aeabi_unwind_cpp_pr0() {
}
