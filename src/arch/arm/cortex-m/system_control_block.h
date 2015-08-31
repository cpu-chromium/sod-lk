#ifndef ARCH_ARM_CORTEX_M_SYSTEM_CONTROL_BLOCK_H
#define ARCH_ARM_CORTEX_M_SYSTEM_CONTROL_BLOCK_H


#include "arch/memory_macros.h"
#include "arch/arm/cortex-m/memory_map.h"

struct SystemControlBlock {
  A_I  uint32_t CPUID;         // 0x000 CPUID Base Register                       
  A_IO uint32_t ICSR;          // 0x004 Interrupt Control and State Register      
  A_IO uint32_t VTOR;          // 0x008 Vector Table Offset Register              
  A_IO uint32_t AIRCR;         // 0x00C Application Interrupt and Reset Control Register
  A_IO uint32_t SCR;           // 0x010 System Control Register                   
  A_IO uint32_t CCR;           // 0x014 Configuration Control Register            
  A_IO uint8_t  SHPR[12];      // 0x018 System Handlers Priority Registers (4-7, 8-11, 12-15)
  A_IO uint32_t SHCSR;         // 0x024 System Handler Control and State Register 
  A_IO uint32_t CFSR;          // 0x028 Configurable Fault Status Register        
  A_IO uint32_t HFSR;          // 0x02C HardFault Status Register                 
  A_IO uint32_t DFSR;          // 0x030 Debug Fault Status Register               
  A_IO uint32_t MMFAR;         // 0x034 MemManage Fault Address Register          
  A_IO uint32_t BFAR;          // 0x038 BusFault Address Register                 
  A_IO uint32_t AFSR;          // 0x03C Auxiliary Fault Status Register           
  A_I  uint32_t ID_PFR[2];     // 0x040 Processor Feature Register                
  A_I  uint32_t ID_DFR;        // 0x048 Debug Feature Register                    
  A_I  uint32_t ID_AFR;        // 0x04C Auxiliary Feature Register                
  A_I  uint32_t ID_MFR[4];     // 0x050 Memory Model Feature Register             
  A_I  uint32_t ID_ISAR[5];    // 0x060 Instruction Set Attributes Register       
       uint32_t RESERVED0[1];
  A_I  uint32_t CLIDR;         // 0x078 Cache Level ID register                   
  A_I  uint32_t CTR;           // 0x07C Cache Type register                       
  A_I  uint32_t CCSIDR;        // 0x080 Cache Size ID Register                    
  A_IO uint32_t CSSELR;        // 0x084 Cache Size Selection Register             
  A_IO uint32_t CPACR;         // 0x088 Coprocessor Access Control Register       
       uint32_t RESERVED3[93];
  A_O  uint32_t STIR;          // 0x200 Software Triggered Interrupt Register     
       uint32_t RESERVED4[15];
  A_I  uint32_t MVFR0;         // 0x240 Media and VFP Feature Register 0          
  A_I  uint32_t MVFR1;         // 0x244 Media and VFP Feature Register 1          
  A_I  uint32_t MVFR2;         // 0x248 Media and VFP Feature Register 1          
       uint32_t RESERVED5[1];
  A_O  uint32_t ICIALLU;       // 0x250 I-Cache Invalidate All to PoU             
       uint32_t RESERVED6[1];
  A_O  uint32_t ICIMVAU;       // 0x258 I-Cache Invalidate by MVA to PoU          
  A_O  uint32_t DCIMVAC;       // 0x25C D-Cache Invalidate by MVA to PoC          
  A_O  uint32_t DCISW;         // 0x260 D-Cache Invalidate by Set-way             
  A_O  uint32_t DCCMVAU;       // 0x264 D-Cache Clean by MVA to PoU               
  A_O  uint32_t DCCMVAC;       // 0x268 D-Cache Clean by MVA to PoC               
  A_O  uint32_t DCCSW;         // 0x26C D-Cache Clean by Set-way                  
  A_O  uint32_t DCCIMVAC;      // 0x270 D-Cache Clean and Invalidate by MVA to PoC
  A_O  uint32_t DCCISW;        // 0x274 D-Cache Clean and Invalidate by Set-way   
       uint32_t RESERVED7[6];
  A_IO uint32_t ITCMCR;        // 0x290 Instruction Tightly-Coupled Memory Control Register
  A_IO uint32_t DTCMCR;        // 0x294 Data Tightly-Coupled Memory Control Registers
  A_IO uint32_t AHBPCR;        // 0x298 AHBP Control Register                     
  A_IO uint32_t CACR;          // 0x29C L1 Cache Control Register                 
  A_IO uint32_t AHBSCR;        // 0x2A0 AHB Slave Control Register                
       uint32_t RESERVED8[1];
  A_IO uint32_t ABFSR;         // 0x2A8 Auxiliary Bus Fault Status Register       
};

#define SYSCTRLBLK ((SystemControlBlock*) (SCS_BASE + 0x0D00UL))

#endif  // ARCH_ARM_CORTEX_M_SYSTEM_CONTROL_BLOCK_H
