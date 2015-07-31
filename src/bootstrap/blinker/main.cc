#include <stdint.h>

#define __I     volatile const       // Defines 'read only' permissions     
#define __O     volatile             // Defines 'write only' permissions    
#define __IO    volatile             // Defines 'read / write' permissions  

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

#define SCS_BASE (0xE000E000UL)    // System Control Space Base Address.
#define SCB_BASE (SCS_BASE +  0x0D00UL) 
#define SCB ((SCB_Type*) SCB_BASE)

#define SCB_CCR_IC_Pos 17
#define SCB_CCR_IC_Msk (1UL << SCB_CCR_IC_Pos)

__attribute__((always_inline)) void __DSB() {
  __asm volatile ("dsb 0xF":::"memory");
}

__attribute__((always_inline)) void __ISB() {
  __asm volatile ("isb 0xF":::"memory");
}

inline void EnableICache() {
  __DSB();
  __ISB();
  SCB->ICIALLU = 0UL;                     // invalidate I-Cache
  SCB->CCR |=  (uint32_t)SCB_CCR_IC_Msk;  // enable I-Cache
  __DSB();
  __ISB();
}

int main() {
  EnableICache();
}

extern "C" void __aeabi_unwind_cpp_pr0() {
}
