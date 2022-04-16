/**
 * @file adc.c
 * @author ZiTe (honmonoh@gmail.com)
 */

#include "adc.h"

void ADC_Channel_Select(ADC_HandleTypeDef *hadc, uint32_t channel);

int ADC_GetValue(ADC_HandleTypeDef *hadc, uint32_t channel, uint16_t *value)
{
  ADC_Channel_Select(hadc, channel);
  HAL_ADC_Start(hadc);
  HAL_StatusTypeDef status = HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
  HAL_ADC_Stop(hadc);
  if (status == HAL_OK)
  {
    *value = (uint16_t)HAL_ADC_GetValue(hadc);
    return 0;
  }
  return -1; /* Error. */
}

void ADC_Channel_Select(ADC_HandleTypeDef *hadc, uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
  {
    // Error_Handler();
  }
}