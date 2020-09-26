//
// Created by adm on 26.09.2020.
//

#ifndef G070_USART_H
#define G070_USART_H

#include "stm32g070xx.h"

#define UART1_RX_SIZE 256

extern uint16_t uart1CountTX;
extern uint16_t uart1Size;
extern uint8_t uart1TX[32];
extern uint16_t uart1CountRX;
extern uint8_t uart1RX[UART1_RX_SIZE];
extern uint8_t uart2RX;

void uart1Send(const uint8_t *in, uint8_t size);

void printS(const char *);

void print8(uint8_t val);

void print16(uint16_t val);

void Configure_USART();

#endif //G070_USART_H
