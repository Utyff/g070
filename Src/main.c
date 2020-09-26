#include <sched.h>
#include "main.h"

void SystemInit(void);

void SystemClock_Config();

void SystemCoreClockUpdate();

void Configure_GPIO_LED(void);

void Delay(uint32_t delay);

uint32_t sysTicks = 0;

uint32_t i = 0;

void initClock();


int main(void) {

    initClock();
    SystemCoreClockUpdate();
    SysTick_Config(64000); // 1 sysTick = 1 ms
    Configure_GPIO_LED();

#pragma ide diagnostic ignored "EndlessLoop"
    while (1) {
        i++;
        GPIOB->ODR ^= GPIO_ODR_OD8;
        Delay(300);
    }
}


void initClock() {
    SET_BIT(FLASH->ACR, FLASH_ACR_PRFTEN);
    SET_BIT(RCC->APBENR2, RCC_APBENR2_SYSCFGEN);
    SET_BIT(RCC->APBENR1, RCC_APBENR1_PWREN);

    // set HSI16 as source and SysClk = 64MHz
    // RCC_PLLCFGR - configure PLL
    //  PLLR = 001 (/2)
    //  PLLN = 000 1000 (x8)
    //  PLLM = 0 (/1)
    //  PLLSRC = 10 (HSI16)
    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_1 | RCC_PLLCFGR_PLLN_3 | RCC_PLLCFGR_PLLR_0 | RCC_PLLCFGR_PLLREN;

    // enable PLL and wait
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // set PLL as SysClk source
    // RCC->CFGR
    //  PPRE = 0
    //  HPRE = 0
    //  SW = 010 (PLLRCLK)
    RCC->CFGR = RCC_CFGR_SW_1;
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);
}
/**
  * @brief  This function :
             - Enables LEDs GPIO clock
             - Configures the Green LED pin on GPIO PA4
             - Configures the orange LED pin on GPIO PB8
  */
void Configure_GPIO_LED(void) {
    // Enable the peripheral clock of GPIOA
//    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

    // Select output mode (01) on PA4 and PB8
//    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER4)) | (GPIO_MODER_MODER4_0);
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE8)) | (GPIO_MODER_MODE8_0);
}


/**
 * @param delay in milliseconds
 */
void Delay(uint32_t delay) {
    uint32_t start = sysTicks;
    while (sysTicks - start < delay);
}

/**
  * @brief  This function handles SysTick Handler.
  */
void SysTick_Handler(void) {
    sysTicks++;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    while (1);
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void) {
    while (1);
}
