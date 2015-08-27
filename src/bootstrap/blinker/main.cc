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

// General Purpose I/O.
struct GPIO_Type {
  __IO uint32_t MODER;    // port mode register,                0x00      
  __IO uint32_t OTYPER;   // port output type register,         0x04      
  __IO uint32_t OSPEEDR;  // port output speed register,        0x08      
  __IO uint32_t PUPDR;    // port pull-up/pull-down register,   0x0C      
  __IO uint32_t IDR;      // port input data register,          0x10      
  __IO uint32_t ODR;      // port output data register,         0x14      
  __IO uint32_t BSRR;     // port bit set/reset register,       0x18      
  __IO uint32_t LCKR;     // port configuration lock register,  0x1C      
  __IO uint32_t AFR[2];   // alternate function registers,      0x20-0x24 
};

// Reset and Clock Control.
struct RCC_Type {
  __IO uint32_t CR;            // RCC clock control register,                          0x00 
  __IO uint32_t PLLCFGR;       // RCC PLL configuration register,                      0x04 
  __IO uint32_t CFGR;          // RCC clock configuration register,                    0x08 
  __IO uint32_t CIR;           // RCC clock interrupt register,                        0x0C 
  __IO uint32_t AHB1RSTR;      // RCC AHB1 peripheral reset register,                  0x10 
  __IO uint32_t AHB2RSTR;      // RCC AHB2 peripheral reset register,                  0x14 
  __IO uint32_t AHB3RSTR;      // RCC AHB3 peripheral reset register,                  0x18 
       uint32_t RESERVED0;     // Reserved, 0x1C                                            
  __IO uint32_t APB1RSTR;      // RCC APB1 peripheral reset register,                  0x20 
  __IO uint32_t APB2RSTR;      // RCC APB2 peripheral reset register,                  0x24 
       uint32_t RESERVED1[2];  // Reserved, 0x28-0x2C                                       
  __IO uint32_t AHB1ENR;       // RCC AHB1 peripheral clock register,                  0x30 
  __IO uint32_t AHB2ENR;       // RCC AHB2 peripheral clock register,                  0x34 
  __IO uint32_t AHB3ENR;       // RCC AHB3 peripheral clock register,                  0x38 
       uint32_t RESERVED2;     // Reserved, 0x3C                                            
  __IO uint32_t APB1ENR;       // RCC APB1 peripheral clock enable register,           0x40 
  __IO uint32_t APB2ENR;       // RCC APB2 peripheral clock enable register,           0x44 
       uint32_t RESERVED3[2];  // Reserved, 0x48-0x4C                                       
  __IO uint32_t AHB1LPENR;     // RCC AHB1 peripheral clock enable in low power mode   0x50 
  __IO uint32_t AHB2LPENR;     // RCC AHB2 peripheral clock enable in low power mode   0x54 
  __IO uint32_t AHB3LPENR;     // RCC AHB3 peripheral clock enable in low power mode   0x58 
       uint32_t RESERVED4;     // Reserved, 0x5C                                            
  __IO uint32_t APB1LPENR;     // RCC APB1 peripheral clock enable in low power mode   0x60 
  __IO uint32_t APB2LPENR;     // RCC APB2 peripheral clock enable in low power mode   0x64 
       uint32_t RESERVED5[2];  // Reserved, 0x68-0x6C                                       
  __IO uint32_t BDCR;          // RCC Backup domain control register,                  0x70 
  __IO uint32_t CSR;           // RCC clock control & status register,                 0x74 
       uint32_t RESERVED6[2];  // Reserved, 0x78-0x7C                                       
  __IO uint32_t SSCGR;         // RCC spread spectrum clock generation register,       0x80 
  __IO uint32_t PLLI2SCFGR;    // RCC PLLI2S configuration register,                   0x84 
  __IO uint32_t PLLSAICFGR;    // RCC PLLSAI configuration register,                   0x88 
  __IO uint32_t DCKCFGR1;      // RCC Dedicated Clocks configuration register1,        0x8C 
  __IO uint32_t DCKCFGR2;      // RCC Dedicated Clocks configuration register 2,       0x90
};

#define  RCC_AHB1ENR_GPIOAEN      ((uint32_t)0x00000001)
#define  RCC_AHB1ENR_GPIOBEN      ((uint32_t)0x00000002)
#define  RCC_AHB1ENR_GPIOCEN      ((uint32_t)0x00000004)
#define  RCC_AHB1ENR_GPIODEN      ((uint32_t)0x00000008)
#define  RCC_AHB1ENR_GPIOEEN      ((uint32_t)0x00000010)
#define  RCC_AHB1ENR_GPIOFEN      ((uint32_t)0x00000020)
#define  RCC_AHB1ENR_GPIOGEN      ((uint32_t)0x00000040)
#define  RCC_AHB1ENR_GPIOHEN      ((uint32_t)0x00000080)
#define  RCC_AHB1ENR_GPIOIEN      ((uint32_t)0x00000100)
#define  RCC_AHB1ENR_GPIOJEN      ((uint32_t)0x00000200)
#define  RCC_AHB1ENR_GPIOKEN      ((uint32_t)0x00000400)
#define  RCC_AHB1ENR_CRCEN        ((uint32_t)0x00001000)
#define  RCC_AHB1ENR_BKPSRAMEN    ((uint32_t)0x00040000)
#define  RCC_AHB1ENR_DTCMRAMEN    ((uint32_t)0x00100000)
#define  RCC_AHB1ENR_DMA1EN       ((uint32_t)0x00200000)
#define  RCC_AHB1ENR_DMA2EN       ((uint32_t)0x00400000)
#define  RCC_AHB1ENR_DMA2DEN      ((uint32_t)0x00800000)
#define  RCC_AHB1ENR_ETHMACEN     ((uint32_t)0x02000000)
#define  RCC_AHB1ENR_ETHMACTXEN   ((uint32_t)0x04000000)
#define  RCC_AHB1ENR_ETHMACRXEN   ((uint32_t)0x08000000)
#define  RCC_AHB1ENR_ETHMACPTPEN  ((uint32_t)0x10000000)
#define  RCC_AHB1ENR_OTGHSEN      ((uint32_t)0x20000000)
#define  RCC_AHB1ENR_OTGHSULPIEN  ((uint32_t)0x40000000)


#define SCS_BASE (0xE000E000UL)    // System Control Space Base Address.
#define SysTick_BASE (SCS_BASE +  0x0010UL)
#define NVIC_BASE (SCS_BASE +  0x0100UL)
#define SCB_BASE (SCS_BASE +  0x0D00UL)

#define PERIPH_BASE       ((uint32_t)0x40000000) // AHB/ABP Peripherals.

// Peripheral memory map.
#define APB1PERIPH_BASE        PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000)

#define GPIOA_BASE        (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASE        (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASE        (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASE        (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASE        (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASE        (AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASE        (AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASE        (AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASE        (AHB1PERIPH_BASE + 0x2000)
#define GPIOJ_BASE        (AHB1PERIPH_BASE + 0x2400)
#define GPIOK_BASE        (AHB1PERIPH_BASE + 0x2800)

#define RCC_BASE           (AHB1PERIPH_BASE + 0x3800)

#define GPIOA     ((GPIO_Type*) GPIOA_BASE)
#define GPIOB     ((GPIO_Type*) GPIOB_BASE)
#define GPIOC     ((GPIO_Type*) GPIOC_BASE)
#define GPIOD     ((GPIO_Type*) GPIOD_BASE)
#define GPIOE     ((GPIO_Type*) GPIOE_BASE)
#define GPIOF     ((GPIO_Type*) GPIOF_BASE)
#define GPIOG     ((GPIO_Type*) GPIOG_BASE)
#define GPIOH     ((GPIO_Type*) GPIOH_BASE)
#define GPIOI     ((GPIO_Type*) GPIOI_BASE)
#define GPIOJ     ((GPIO_Type*) GPIOJ_BASE)
#define GPIOK     ((GPIO_Type*) GPIOK_BASE)

#define SCB ((SCB_Type*) SCB_BASE)
#define SysTick ((SysTick_Type *) SysTick_BASE) 
#define NVIC ((NVIC_Type*) NVIC_BASE)
#define RCC  ((RCC_Type*) RCC_BASE)

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
volatile uint32_t loop_count = 0;

extern "C" void SysTick_Handler() {
  ++milisecs_count;
}

void DelayMS(uint32_t wait_ms) {
  auto start_ms = milisecs_count;
  while ((milisecs_count - start_ms) < wait_ms) {
    __WFI();
  }
}

volatile void SetBit(volatile uint32_t& dest, uint32_t bit) {
  dest |= bit;
}

bool ReadBit(const volatile uint32_t& reg, uint32_t bit) {
  return reg & bit;
}

inline bool GPIOA_EnableClock() {
  volatile uint32_t tmpreg;
  SetBit(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
  // Delay after an RCC peripheral clock enabling.
  return ReadBit(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
}

struct GPIO_InitType {
  uint32_t Pin;
  uint32_t Mode;
  uint32_t Pull;  
  uint32_t Speed; 
  uint32_t Alternate;  
};

#define GPIO_PIN_0                 ((uint16_t)0x0001)  /* Pin 0 selected    */
#define GPIO_PIN_1                 ((uint16_t)0x0002)  /* Pin 1 selected    */
#define GPIO_PIN_2                 ((uint16_t)0x0004)  /* Pin 2 selected    */
#define GPIO_PIN_3                 ((uint16_t)0x0008)  /* Pin 3 selected    */
#define GPIO_PIN_4                 ((uint16_t)0x0010)  /* Pin 4 selected    */
#define GPIO_PIN_5                 ((uint16_t)0x0020)  /* Pin 5 selected    */
#define GPIO_PIN_6                 ((uint16_t)0x0040)  /* Pin 6 selected    */
#define GPIO_PIN_7                 ((uint16_t)0x0080)  /* Pin 7 selected    */
#define GPIO_PIN_8                 ((uint16_t)0x0100)  /* Pin 8 selected    */
#define GPIO_PIN_9                 ((uint16_t)0x0200)  /* Pin 9 selected    */
#define GPIO_PIN_10                ((uint16_t)0x0400)  /* Pin 10 selected   */
#define GPIO_PIN_11                ((uint16_t)0x0800)  /* Pin 11 selected   */
#define GPIO_PIN_12                ((uint16_t)0x1000)  /* Pin 12 selected   */
#define GPIO_PIN_13                ((uint16_t)0x2000)  /* Pin 13 selected   */
#define GPIO_PIN_14                ((uint16_t)0x4000)  /* Pin 14 selected   */
#define GPIO_PIN_15                ((uint16_t)0x8000)  /* Pin 15 selected   */
#define GPIO_PIN_All               ((uint16_t)0xFFFF)  /* All pins selected */

#define  GPIO_MODE_INPUT                        ((uint32_t)0x00000000)   /*!< Input Floating Mode                   */
#define  GPIO_MODE_OUTPUT_PP                    ((uint32_t)0x00000001)   /*!< Output Push Pull Mode                 */
#define  GPIO_MODE_OUTPUT_OD                    ((uint32_t)0x00000011)   /*!< Output Open Drain Mode                */
#define  GPIO_MODE_AF_PP                        ((uint32_t)0x00000002)   /*!< Alternate Function Push Pull Mode     */
#define  GPIO_MODE_AF_OD                        ((uint32_t)0x00000012)   /*!< Alternate Function Open Drain Mode    */

#define  GPIO_MODE_ANALOG                       ((uint32_t)0x00000003)   /*!< Analog Mode  */
    
#define  GPIO_MODE_IT_RISING                    ((uint32_t)0x10110000)   /*!< External Interrupt Mode with Rising edge trigger detection          */
#define  GPIO_MODE_IT_FALLING                   ((uint32_t)0x10210000)   /*!< External Interrupt Mode with Falling edge trigger detection         */
#define  GPIO_MODE_IT_RISING_FALLING            ((uint32_t)0x10310000)   /*!< External Interrupt Mode with Rising/Falling edge trigger detection  */
 
#define  GPIO_MODE_EVT_RISING                   ((uint32_t)0x10120000)   /*!< External Event Mode with Rising edge trigger detection               */
#define  GPIO_MODE_EVT_FALLING                  ((uint32_t)0x10220000)   /*!< External Event Mode with Falling edge trigger detection              */
#define  GPIO_MODE_EVT_RISING_FALLING           ((uint32_t)0x10320000)   /*!< External Event Mode with Rising/Falling edge trigger detection       */

#define  GPIO_SPEED_LOW         ((uint32_t)0x00000000)  /*!< Low speed     */
#define  GPIO_SPEED_MEDIUM      ((uint32_t)0x00000001)  /*!< Medium speed  */
#define  GPIO_SPEED_FAST        ((uint32_t)0x00000002)  /*!< Fast speed    */
#define  GPIO_SPEED_HIGH        ((uint32_t)0x00000003)  /*!< High speed    */

#define  GPIO_NOPULL        ((uint32_t)0x00000000)   /*!< No Pull-up or Pull-down activation  */
#define  GPIO_PULLUP        ((uint32_t)0x00000001)   /*!< Pull-up activation                  */
#define  GPIO_PULLDOWN      ((uint32_t)0x00000002)   /*!< Pull-down activation                */

#define GPIO_MODER_MODER0                    ((uint32_t)0x00000003)
#define GPIO_OSPEEDER_OSPEEDR0               ((uint32_t)0x00000003)
#define GPIO_PUPDR_PUPDR0                    ((uint32_t)0x00000003)

#define EXTI_MODE             ((uint32_t)0x10000000)
#define GPIO_MODE             ((uint32_t)0x00000003)
#define GPIO_OUTPUT_TYPE      ((uint32_t)0x00000010)

#define GPIO_OTYPER_OT_0                     ((uint32_t)0x00000001)

// Initializes the GPIOx peripheral according to the specified parameters in the GPIO_Init.
// GPIOx: where x can be (A..K) to select the GPIO peripheral.
// GPIO_Init: pointer to a GPIO_InitTypeDef structure that contains
// the configuration information for the specified GPIO peripheral.
void GPIO_InitPin(GPIO_Type* GPIOx, GPIO_InitType* GPIO_Init) {
  uint32_t temp = 0;

  /* Configure the port pins */
  for(uint32_t position = 0; position < 16UL; position++) {
    /* Get the IO position */
    uint32_t ioposition = ((uint32_t)0x01) << position;
    /* Get the current IO position */
    uint32_t iocurrent = (uint32_t)(GPIO_Init->Pin) & ioposition;

    if(iocurrent == ioposition) {
      /*--------------------- GPIO Mode Configuration ------------------------*/
      /* In case of Alternate function mode selection */
      if((GPIO_Init->Mode == GPIO_MODE_AF_PP) || (GPIO_Init->Mode == GPIO_MODE_AF_OD)) {        
        /* Configure Alternate function mapped with the current IO */
        temp = GPIOx->AFR[position >> 3];
        temp &= ~((uint32_t)0xF << ((uint32_t)(position & (uint32_t)0x07) * 4)) ;
        temp |= ((uint32_t)(GPIO_Init->Alternate) << (((uint32_t)position & (uint32_t)0x07) * 4));
        GPIOx->AFR[position >> 3] = temp;
      }

      /* Configure IO Direction mode (Input, Output, Alternate or Analog) */
      temp = GPIOx->MODER;
      temp &= ~(GPIO_MODER_MODER0 << (position * 2));
      temp |= ((GPIO_Init->Mode & GPIO_MODE) << (position * 2));
      GPIOx->MODER = temp;

      /* In case of Output or Alternate function mode selection */
      if((GPIO_Init->Mode == GPIO_MODE_OUTPUT_PP) || (GPIO_Init->Mode == GPIO_MODE_AF_PP) ||
         (GPIO_Init->Mode == GPIO_MODE_OUTPUT_OD) || (GPIO_Init->Mode == GPIO_MODE_AF_OD)) {
        /* Configure the IO Speed */
        temp = GPIOx->OSPEEDR; 
        temp &= ~(GPIO_OSPEEDER_OSPEEDR0 << (position * 2));
        temp |= (GPIO_Init->Speed << (position * 2));
        GPIOx->OSPEEDR = temp;

        /* Configure the IO Output Type */
        temp = GPIOx->OTYPER;
        temp &= ~(GPIO_OTYPER_OT_0 << position) ;
        temp |= (((GPIO_Init->Mode & GPIO_OUTPUT_TYPE) >> 4) << position);
        GPIOx->OTYPER = temp;
      }

      /* Activate the Pull-up or Pull down resistor for the current IO */
      temp = GPIOx->PUPDR;
      temp &= ~(GPIO_PUPDR_PUPDR0 << (position * 2));
      temp |= ((GPIO_Init->Pull) << (position * 2));
      GPIOx->PUPDR = temp;

      /*--------------------- EXTI Mode Configuration ------------------------*/
      /* Configure the External Interrupt or event for the current IO */
      if((GPIO_Init->Mode & EXTI_MODE) == EXTI_MODE){
        // Not implemented yet.
        halt_bp();
      }
    }
  }
}


inline void GPIO_TogglePin(GPIO_Type* GPIOx, uint16_t GPIO_Pin) {
  GPIOx->ODR ^= GPIO_Pin;
}


int main() {
  EnableICache();
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  GPIOA_EnableClock();

  GPIO_InitType GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitPin(GPIOA, &GPIO_InitStruct);


  if (!InitSysTick(16000))
    halt_bp();
  while (true) {
    GPIO_TogglePin(GPIOA, 9);
    DelayMS(500);
    ++loop_count;
  }
}

extern "C" void __aeabi_unwind_cpp_pr0() {
}
