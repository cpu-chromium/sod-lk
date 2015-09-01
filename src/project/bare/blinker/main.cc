#include <stdint.h>
#include "arch/memory_macros.h"
#include "arch/arm/cortex-m/interrupt_types.h"
#include "arch/arm/cortex-m/system_control_block.h"
#include "arch/arm/cortex-m/reset_clock_control_block.h"
#include "arch/arm/cortex-m/nested_interrupt_controller.h"

// System Timer (SysTick).
struct SysTick_Type {
  A_IO uint32_t CTRL;             // 0x000 (R/W)  SysTick Control and Status Register
  A_IO uint32_t LOAD;             // 0x004 (R/W)  SysTick Reload Value Register
  A_IO uint32_t VAL;              // 0x008 (R/W)  SysTick Current Value Register
  A_I  uint32_t CALIB;            // 0x00C (R/ )  SysTick Calibration Register
};

// General Purpose I/O.
struct GPIO_Type {
  A_IO uint32_t MODER;    // port mode register,                0x00      
  A_IO uint32_t OTYPER;   // port output type register,         0x04      
  A_IO uint32_t OSPEEDR;  // port output speed register,        0x08      
  A_IO uint32_t PUPDR;    // port pull-up/pull-down register,   0x0C      
  A_IO uint32_t IDR;      // port input data register,          0x10      
  A_IO uint32_t ODR;      // port output data register,         0x14      
  A_IO uint32_t BSRR;     // port bit set/reset register,       0x18      
  A_IO uint32_t LCKR;     // port configuration lock register,  0x1C      
  A_IO uint32_t AFR[2];   // alternate function registers,      0x20-0x24 
};

#define SysTick_BASE (SCS_BASE +  0x0010UL)

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

#define SysTick ((SysTick_Type *) SysTick_BASE) 

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


__attribute__((always_inline)) void __WFI() {
  __asm volatile ("wfi");
}

#define __BKPT(value) __asm volatile ("bkpt "#value)

void halt_bp() {
  __BKPT(6);  
}

// System Tick Configuration
//   The function initializes the System Timer and its interrupt, and starts the System Tick Timer.
//   Counter is in free running mode to generate periodic interrupts.
//   |ticks|  Number of ticks between two interrupts.

inline bool SysTick_Config(uint32_t ticks) {
  if ((ticks - 1UL) > SysTick_LOAD_RELOAD_Msk)
    return false;    // Reload value impossible.

  SysTick->LOAD  = (uint32_t)(ticks - 1UL);
  NVIC_SetPriority(SysTick_IRQn, (1UL << NVIC_PRIO_BITS) - 1UL);
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
  SetBit(RSTCLKBLK->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
  // Delay after an RCC peripheral clock enabling.
  return ReadBit(RSTCLKBLK->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
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
  EnableDCache();
  SetPriorityGrouping(PRIORITYGROUP_4);
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
