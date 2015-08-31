#ifndef ARCH_ARM_CORTEX_M_RESET_CLOCK_CONTROL_BLOCK_H
#define ARCH_ARM_CORTEX_M_RESET_CLOCK_CONTROL_BLOCK_H

// Reset and Clock Control.
struct ResetClockControlBlock {
  A_IO uint32_t CR;            // RCC clock control register,                          0x00 
  A_IO uint32_t PLLCFGR;       // RCC PLL configuration register,                      0x04 
  A_IO uint32_t CFGR;          // RCC clock configuration register,                    0x08 
  A_IO uint32_t CIR;           // RCC clock interrupt register,                        0x0C 
  A_IO uint32_t AHB1RSTR;      // RCC AHB1 peripheral reset register,                  0x10 
  A_IO uint32_t AHB2RSTR;      // RCC AHB2 peripheral reset register,                  0x14 
  A_IO uint32_t AHB3RSTR;      // RCC AHB3 peripheral reset register,                  0x18 
       uint32_t RESERVED0;     // Reserved, 0x1C                                            
  A_IO uint32_t APB1RSTR;      // RCC APB1 peripheral reset register,                  0x20 
  A_IO uint32_t APB2RSTR;      // RCC APB2 peripheral reset register,                  0x24 
       uint32_t RESERVED1[2];  // Reserved, 0x28-0x2C                                       
  A_IO uint32_t AHB1ENR;       // RCC AHB1 peripheral clock register,                  0x30 
  A_IO uint32_t AHB2ENR;       // RCC AHB2 peripheral clock register,                  0x34 
  A_IO uint32_t AHB3ENR;       // RCC AHB3 peripheral clock register,                  0x38 
       uint32_t RESERVED2;     // Reserved, 0x3C                                            
  A_IO uint32_t APB1ENR;       // RCC APB1 peripheral clock enable register,           0x40 
  A_IO uint32_t APB2ENR;       // RCC APB2 peripheral clock enable register,           0x44 
       uint32_t RESERVED3[2];  // Reserved, 0x48-0x4C                                       
  A_IO uint32_t AHB1LPENR;     // RCC AHB1 peripheral clock enable in low power mode   0x50 
  A_IO uint32_t AHB2LPENR;     // RCC AHB2 peripheral clock enable in low power mode   0x54 
  A_IO uint32_t AHB3LPENR;     // RCC AHB3 peripheral clock enable in low power mode   0x58 
       uint32_t RESERVED4;     // Reserved, 0x5C                                            
  A_IO uint32_t APB1LPENR;     // RCC APB1 peripheral clock enable in low power mode   0x60 
  A_IO uint32_t APB2LPENR;     // RCC APB2 peripheral clock enable in low power mode   0x64 
       uint32_t RESERVED5[2];  // Reserved, 0x68-0x6C                                       
  A_IO uint32_t BDCR;          // RCC Backup domain control register,                  0x70 
  A_IO uint32_t CSR;           // RCC clock control & status register,                 0x74 
       uint32_t RESERVED6[2];  // Reserved, 0x78-0x7C                                       
  A_IO uint32_t SSCGR;         // RCC spread spectrum clock generation register,       0x80 
  A_IO uint32_t PLLI2SCFGR;    // RCC PLLI2S configuration register,                   0x84 
  A_IO uint32_t PLLSAICFGR;    // RCC PLLSAI configuration register,                   0x88 
  A_IO uint32_t DCKCFGR1;      // RCC Dedicated Clocks configuration register1,        0x8C 
  A_IO uint32_t DCKCFGR2;      // RCC Dedicated Clocks configuration register 2,       0x90
};

#define  RCC_AHB1ENR_GPIOAEN      (0x00000001UL)
#define  RCC_AHB1ENR_GPIOBEN      (0x00000002UL)
#define  RCC_AHB1ENR_GPIOCEN      (0x00000004UL)
#define  RCC_AHB1ENR_GPIODEN      (0x00000008UL)
#define  RCC_AHB1ENR_GPIOEEN      (0x00000010UL)
#define  RCC_AHB1ENR_GPIOFEN      (0x00000020UL)
#define  RCC_AHB1ENR_GPIOGEN      (0x00000040UL)
#define  RCC_AHB1ENR_GPIOHEN      (0x00000080UL)
#define  RCC_AHB1ENR_GPIOIEN      (0x00000100UL)
#define  RCC_AHB1ENR_GPIOJEN      (0x00000200UL)
#define  RCC_AHB1ENR_GPIOKEN      (0x00000400UL)
#define  RCC_AHB1ENR_CRCEN        (0x00001000UL)
#define  RCC_AHB1ENR_BKPSRAMEN    (0x00040000UL)
#define  RCC_AHB1ENR_DTCMRAMEN    (0x00100000UL)
#define  RCC_AHB1ENR_DMA1EN       (0x00200000UL)
#define  RCC_AHB1ENR_DMA2EN       (0x00400000UL)
#define  RCC_AHB1ENR_DMA2DEN      (0x00800000UL)
#define  RCC_AHB1ENR_ETHMACEN     (0x02000000UL)
#define  RCC_AHB1ENR_ETHMACTXEN   (0x04000000UL)
#define  RCC_AHB1ENR_ETHMACRXEN   (0x08000000UL)
#define  RCC_AHB1ENR_ETHMACPTPEN  (0x10000000UL)
#define  RCC_AHB1ENR_OTGHSEN      (0x20000000UL)
#define  RCC_AHB1ENR_OTGHSULPIEN  (0x40000000UL)

#define RSTCLKBLK  ((ResetClockControlBlock*) AHB1PERIPH_BASE + 0x3800UL)

#endif  // ARCH_ARM_CORTEX_M_RESET_CLOCK_CONTROL_BLOCK_H
