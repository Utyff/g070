#include <sched.h>
#include "main.h"
#include "usart.h"

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
    Configure_USART();

#pragma ide diagnostic ignored "EndlessLoop"
    while (1) {
        i++;
        GPIOB->ODR ^= GPIO_ODR_OD8;
        GPIOB->ODR ^= GPIO_ODR_OD5;

        printS("\n\rsystick: ");
        print16(sysTicks);

        Delay(300);
        GPIOB->ODR ^= GPIO_ODR_OD5;
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
    // Enable PLLR Clock output.
    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_1 | RCC_PLLCFGR_PLLN_3 | RCC_PLLCFGR_PLLR_0 | RCC_PLLCFGR_PLLREN;

    // enable PLL
    RCC->CR |= RCC_CR_PLLON;
    // wait PLL ready
    while (!(RCC->CR & RCC_CR_PLLRDY));

    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, (FLASH_ACR_LATENCY_1));

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
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

    // Select output mode (01) on PA4 and PB8
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE5)) | (GPIO_MODER_MODE5_0);
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE8)) | (GPIO_MODER_MODE8_0);
}


/**
 * @param delay in milliseconds
 */
void Delay(uint32_t delay) {
    uint32_t start = sysTicks;
    while (sysTicks - start < delay);
}

void _strcpy(uint8_t *dst, const uint8_t *src) {
    int i = 0;
    do {
        dst[i] = src[i];
    } while (src[i++] != 0);
}

void _memcpy(uint8_t *dst, const uint8_t *src, uint16_t size) {
    for (int i = 0; i < size; i++) {
        dst[i] = src[i];
    }
}

void _memset(uint8_t *dst, const uint8_t src, uint16_t size) {
    for (int i = 0; i < size; i++) {
        dst[i] = src;
    }
}

void _itoa(uint16_t i, char *p) {
//    char const digit[] = "0123456789";
    if (i < 0) {
        *p++ = '-';
        i *= -1;
    }

    int shifter = i;
    // Move to where representation ends
    do {
        ++p;
        shifter = shifter / 10;
    } while (shifter);
    *p = '\0';

    // Move back, inserting digits as u go
    do {
        *--p = (char) ('0' + i % 10);
        i = i / (uint16_t) 10;
    } while (i);
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
