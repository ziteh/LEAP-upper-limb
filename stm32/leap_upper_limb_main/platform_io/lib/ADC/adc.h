/**
 * @file adc.h
 * @author ZiTe (honmonoh@gmail.com)
 */

#ifndef __ADC_H
#define __ADC_H

#include "stm32f4xx_hal.h"

int ADC_GetValue(ADC_HandleTypeDef *hadc, uint32_t channel, uint16_t *value);

#endif /* __ADC_H */