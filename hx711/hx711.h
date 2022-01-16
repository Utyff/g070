#ifndef HX711_H_
#define HX711_H_


#include "main.h"


typedef struct {
    int32_t offset;
    float coef;
    uint8_t lock;
} hx711_t;

void        hx711_init(hx711_t *hx711);
int32_t     hx711_value();
int32_t     hx711_value_ave(hx711_t *hx711, uint16_t sample);

void        hx711_coef_set(hx711_t *hx711, float coef);
float       hx711_coef_get(hx711_t *hx711);
void        hx711_calibration(hx711_t *hx711, int32_t value_noload, int32_t value_load, float scale);
void        hx711_tare(hx711_t *hx711, uint16_t sample);
float       hx711_weight(hx711_t *hx711, uint16_t sample);
void        hx711_power_down(hx711_t *hx711);
void        hx711_power_up(hx711_t *hx711);

#endif
