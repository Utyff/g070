#include "main.h"


void SystemClock_Config(void);

uint32_t i = 0;


int main(void) {

  #pragma ide diagnostic ignored "EndlessLoop"
  while (1) {
      i++;
//      HAL_GPIO_TogglePin(GPIOB,  GPIO_PIN_8);
//      HAL_GPIO_WritePin(GPIOB,  GPIO_PIN_5, GPIO_PIN_SET);
//      HAL_Delay(300);
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
}
