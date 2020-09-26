#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g070xx.h"

void Error_Handler(void);

#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOB

#ifdef __cplusplus
}
#endif


typedef enum
{
    RESET = 0,
    SET = !RESET
} FlagStatus, ITStatus;

typedef enum
{
    DISABLE = 0,
    ENABLE = !DISABLE
} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum
{
    SUCCESS = 0,
    ERROR = !SUCCESS
} ErrorStatus;

#define UNUSED(X) (void)X      /* To avoid gcc/g++ warnings */

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

/*#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))*/

void _strcpy(uint8_t *dst, const uint8_t *src);
void _memcpy(uint8_t *dst, const uint8_t *src, uint16_t size);
void _memset(uint8_t *dst, const uint8_t src, uint16_t size);
void _itoa(uint16_t i, char *p);

#endif /* __MAIN_H */
