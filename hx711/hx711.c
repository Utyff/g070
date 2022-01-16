#include "hx711.h"

#define _HX711_DELAY_US_LOOP  3
#define HX711_GPIO_CLK     GPIOB
#define HX711_SET_CLK      HX711_GPIO_CLK->BSRR = GPIO_BSRR_BS4
#define HX711_RESET_CLK    HX711_GPIO_CLK->BRR = GPIO_BRR_BR4
#define HX711_GPIO_DATA    GPIOB
#define HX711_GET_DATA     (HX711_GPIO_DATA->IDR & GPIO_ODR_OD3)
#define hx711_delay(x)     Delay(x)

#ifdef HX711_DEBUG
#define T_SIZE 128
int iO = 0;
uint32_t dataO[T_SIZE] = {0};
uint32_t timeO[T_SIZE] = {0};
#endif

// Delay per 1 us.
void hx711_delay_us(void) {
    uint32_t delay = _HX711_DELAY_US_LOOP;
    while (delay > 0) {
        delay--;
        __NOP(); __NOP(); __NOP(); __NOP();
    }
}

void hx711_init(hx711_t *hx711) {
    // Select input mode (00) on PB3 for HX711_DATA
    HX711_GPIO_DATA->MODER = (HX711_GPIO_DATA->MODER & ~(GPIO_MODER_MODE3));
    // Select output mode (01) on PB4 for HX711_CLK
    HX711_GPIO_CLK->MODER = (HX711_GPIO_CLK->MODER & ~(GPIO_MODER_MODE4)) | (GPIO_MODER_MODE4_0);

#ifdef HX711_DEBUG
    // Select output mode (01) on PA6 and PA7 for DEBUG
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE6)) | (GPIO_MODER_MODE6_0);
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE7)) | (GPIO_MODER_MODE7_0);
    GPIOA->BSRR = GPIO_BSRR_BS6;
#endif

    HX711_SET_CLK;
    hx711_delay(10);
    HX711_RESET_CLK;
    hx711_delay(10);

    hx711_value(hx711);
    hx711_value(hx711);

//    GPIOA->BRR = GPIO_BRR_BR6;
}

#ifdef HX711_DEBUG
uint32_t getData() {
    uint32_t d = HX711_GET_DATA;
    if (iO < T_SIZE) {
        dataO[iO] = d;
        timeO[iO] = sysTicks;
        iO++;
    }
    return d;
}
#endif

int32_t hx711_value() {
    uint32_t data = 0;
    uint32_t startTime = sysTicks;

//    GPIOA->BSRR = GPIO_BSRR_BS7;

    // wait for 0 - data ready
    while (HX711_GET_DATA) {
        hx711_delay(1);
        if (sysTicks - startTime > 150) {
            return 0;
        }
    }

    for (int8_t i = 0; i < 24; i++) {
        HX711_SET_CLK;
        hx711_delay_us();
        HX711_RESET_CLK;
        hx711_delay_us();

        data = data << 1;
        if (HX711_GET_DATA) {
            data++;
        }
    }

    data = data ^ 0x800000;

    // add one CLK pulse:
    // Input channel - A
    // Gain - 128
    HX711_SET_CLK;
    hx711_delay_us();
    HX711_RESET_CLK;
    hx711_delay_us();

//    GPIOA->BRR = GPIO_BRR_BR7;

    return data;
}

int32_t data = 0;

float hx711_weight(hx711_t *hx711, uint16_t sample) {
    int64_t ave = 0;
    for (uint16_t i = 0; i < sample; i++) {
        ave += hx711_value();
        hx711_delay(5);
    }
    data = (int32_t) (ave / sample);
    float answer = (data - hx711->offset) / hx711->coef;

    return answer;
}

/*
int32_t hx711_value_ave(hx711_t *hx711, uint16_t sample)
{
  hx711_lock(hx711);
  int64_t  ave = 0;
  for(uint16_t i=0 ; i<sample ; i++)
  {
    ave += hx711_value();
    hx711_delay(5);
  }
  int32_t answer = (int32_t)(ave / sample);
  hx711_unlock(hx711);
  return answer;
} */

//void hx711_tare(hx711_t *hx711, uint16_t sample) {
//    hx711_lock(hx711);
//    int64_t ave = 0;
//    for (uint16_t i = 0; i < sample; i++) {
//        ave += hx711_value();
//        hx711_delay(5);
//    }
//    hx711->offset = (int32_t) (ave / sample);
//    hx711_unlock(hx711);
//}

void hx711_calibration(hx711_t *hx711, int32_t noload_raw, int32_t load_raw, float scale) {
    hx711->offset = noload_raw;
    hx711->coef = (load_raw - noload_raw) / scale;
}

void hx711_coef_set(hx711_t *hx711, float coef) {
    hx711->coef = coef;
}
/*
void hx711_power_down() {
  HX711_RESET_CLK;
  HX711_SET_CLK;
  hx711_delay(1);
}

void hx711_power_up() {
  HX711_RESET_CLK;
}
*/