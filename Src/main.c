#include "main.h"

void SystemInit(void);

void SystemClock_Config();

void SystemCoreClockUpdate();

void Configure_GPIO_LED(void);

void Delay(uint32_t delay);

uint32_t sysTicks = 0;

uint32_t i = 0;


int main(void) {
    SystemInit();
    SystemCoreClockUpdate();
    SysTick_Config(16000); // 1 sysTick = 1 ms
    Configure_GPIO_LED();

#pragma ide diagnostic ignored "EndlessLoop"
    while (1) {
        i++;
        GPIOB->ODR ^= GPIO_ODR_OD8;
        Delay(300);
    }
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
