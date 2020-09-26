#include "main.h"
#include "usart.h"

uint16_t uart1CountTX = 0;
uint16_t uart1Size = 0;
uint8_t uart1TX[32];
uint16_t uart1CountRX = 0;
uint8_t uart1RX[UART1_RX_SIZE];
uint16_t send2 = 0;
uint8_t string2send2[32] = "STm\n";
uint8_t uart2RX = 0;

#define hex2char(hex) (uint8_t)((hex)<=9u ? (hex) + '0' : (hex) + 'a' - 10u)

void print16(uint16_t val) {
    uint8_t buf[5];

    buf[0] = hex2char(val >> 12u & 0xFu);
    buf[1] = hex2char(val >> 8u & 0xFu);
    buf[2] = hex2char(val >> 4u & 0xFu);
    buf[3] = hex2char(val & 0xFu);
    buf[4] = 0;

    printS((char *) buf);
}

void print8(uint8_t val) {
    uint8_t buf[3];

    buf[0] = hex2char(val >> 4u & 0xFu);
    buf[1] = hex2char(val & 0xFu);
    buf[2] = 0;

    printS((char *) buf);
}

void printS(const char *str) {
    // wait till end current transmission
    while (send2 != 0);

    _strcpy(string2send2, (uint8_t *) str);
    // start USART transmission. Will initiate TC if TXE
    USART2->TDR = string2send2[send2];
    send2 = 1;
}

void uart1Send(const uint8_t *in, uint8_t size) {
    // wait till end current transmission
    while (uart1CountTX != 0);

    uart1Size = size;
    _memcpy(uart1TX, in, size);
    // start USART transmission. Will initiate TC if TXE
    USART1->TDR = uart1TX[uart1CountTX];
    uart1CountTX = 1;
}

/**
  * @brief  This function :
             - Enables GPIO clock
             - Configures the USART1 pins on GPIO PA9 PA10
             - Configures the USART2 pins on GPIO PA2 PA3
  */
void Configure_GPIO_USART(void) {
    // Enable the peripheral clock of GPIOA
//    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

    // GPIO configuration for USART1 signals
    // (1) Select AF mode (01) on PA9 and PA10
    // (2) AF1 for USART1 signals
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE9 | GPIO_MODER_MODE10))
                   | (GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1); // (1)
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(GPIO_AFRH_AFSEL9 | GPIO_AFRH_AFSEL10))
                    | GPIO_AFRH_AFSEL9_0 | GPIO_AFRH_AFSEL10_0; // (2)

    // GPIO configuration for USART2 signals
    // (1) Select AF mode (01) on PA2 and PA3
    // (2) AF1 for USART2 signals
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3))
                   | (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1); // (1)
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3))
                    | GPIO_AFRL_AFSEL2_0 | GPIO_AFRL_AFSEL3_0; // (2)
}

/**
  * @brief  This function configures USART1.
  */
#define APBCLK 64000000UL
#define SYSCLK 64000000UL
#define BAUDRATE1 9600UL

void Configure_USART1(void) {
    // Enable the peripheral clock USART1
    RCC->APBENR2 |= RCC_APBENR2_USART1EN;

    // Configure USART1
    // (1) oversampling by 16, 115200 baud
    // (2) 8 data bit, 1 start bit, 1 stop bit, no parity

    // for system clock (SYSCLK) selected as USART1 clock
//    RCC->CFGR3 &= ~RCC_CFGR3_USART1SW;
//    RCC->CFGR3 |= RCC_CFGR3_USART1SW_0;
//    USART1->BRR = (SYSCLK + BAUDRATE1 / 2) / BAUDRATE1; // скорость usart
    // for APB clock selected as USART1 clock
    USART1->BRR = APBCLK / BAUDRATE1;

    USART1->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
    USART1->CR2 = USART_CR1_RXNEIE_RXFNEIE;

    // polling idle frame Transmission
    while ((USART1->ISR & USART_ISR_TC) != USART_ISR_TC) {
        // add time out here for a robust application
    }
    USART1->ICR |= USART_ICR_TCCF; // clear TC flag
    USART1->CR1 |= USART_CR1_TCIE; // enable TC interrupt

    // Configure IT
    // (3) Set priority for USART1_IRQn
    // (4) Enable USART1_IRQn
    NVIC_SetPriority(USART1_IRQn, 0); // (3)
    NVIC_EnableIRQ(USART1_IRQn); // (4)
}

/**
  * @brief  This function configures USART2.
  */
void Configure_USART2(void) {
    // Enable the peripheral clock USART2
    RCC->APBENR1 |= RCC_APBENR1_USART2EN;

    // Configure USART2
    // (1) oversampling by 16, 115200 baud
    // (2) 8 data bit, 1 start bit, 1 stop bit, no parity
    USART2->BRR = 640000 / 1152; // (1)
    USART2->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE; // (2)
    USART2->CR2 = USART_CR1_RXNEIE_RXFNEIE;

    // polling idle frame Transmission
    while ((USART2->ISR & USART_ISR_TC) != USART_ISR_TC) {
        // add time out here for a robust application
    }
    USART2->ICR |= USART_ICR_TCCF; // clear TC flag
    USART2->CR1 |= USART_CR1_TCIE; // enable TC interrupt

    // Configure IT
    // (3) Set priority for USART2_IRQn
    // (4) Enable USART2_IRQn
    NVIC_SetPriority(USART2_IRQn, 0); // (3)
    NVIC_EnableIRQ(USART2_IRQn); // (4)
}

/**
  * @brief  This function handles USART1 interrupt request.
  */
void USART1_IRQHandler(void) {
    if ((USART1->ISR & USART_ISR_TC) == USART_ISR_TC) {
        if (uart1CountTX == uart1Size) {
            uart1CountTX = 0;
            USART1->ICR |= USART_ICR_TCCF; // Clear transfer complete flag
        } else {
            // clear transfer complete flag and fill TDR with a new char
            USART1->TDR = uart1TX[uart1CountTX++];
        }
    }
    if ((USART1->ISR & USART_ISR_RXNE_RXFNE) == USART_ISR_RXNE_RXFNE) {
        uart1RX[uart1CountRX] = (uint8_t) (USART1->RDR); // Receive data, clear flag
        if (++uart1CountRX >= UART1_RX_SIZE) {
            uart1CountRX = 0;
        }
    }
}

/**
  * @brief  This function handles USART2 interrupt request.
  */
void USART2_IRQHandler(void) {
    if ((USART2->ISR & USART_ISR_TC) == USART_ISR_TC) {
        if (string2send2[send2] == 0) {
            send2 = 0;
            USART2->ICR |= USART_ICR_TCCF; // Clear transfer complete flag
        } else {
            // clear transfer complete flag and fill TDR with a new char
            USART2->TDR = string2send2[send2++];
        }
    }
    if ((USART2->ISR & USART_ISR_RXNE_RXFNE) == USART_ISR_RXNE_RXFNE) {
        uart2RX = (uint8_t) (USART2->RDR); // Receive data, clear flag
    }
}

void Configure_USART(void) {
    Configure_GPIO_USART();
    Configure_USART1();
    Configure_USART2();
}
