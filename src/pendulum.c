#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "uart.h"
#include "pendulum.h"

static ADC_HandleTypeDef hadc1;
static ADC_ChannelConfTypeDef sConfigADC;

void pendulum_init(void)
{
/* TODO: Enable ADC1 clock */
        __HAL_RCC_ADC1_CLK_ENABLE();

/* TODO: Enable GPIOA clock */
	    __HAL_RCC_GPIOA_CLK_ENABLE();

/* TODO: Initialise PB0 with:
- Pin 0
- Analog mode
- No pull
- High frequency */

GPIO_InitTypeDef GPIO_InitStruct;               //  i hooked up a voltage sensor to PB0;
GPIO_InitStruct.Pin = GPIO_PIN_0;
GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

// GPIO_InitTypeDef GPIO_InitStruct;               // PA4 - analog and because i hooked up a voltage sensor to PB0 , has same channel and everything!
// GPIO_InitStruct.Pin = GPIO_PIN_4;
// GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
// GPIO_InitStruct.Pull = GPIO_NOPULL;
// GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
// HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* TODO: Initialise ADC1 with:
- Instance ADC 1
- Div 2 prescaler
- 12 bit resolution
- Data align right
- Continuous conversion mode disabled
- Number of conversions = 1
Note: Configuration parameters not mentioned above
are not required for this lab. */

    hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;

/* TODO: Configure ADC channel to:
- Channel 8
- Rank 1
- Sampling time 480 cycles
- Offset 0 */
	sConfigADC.Channel = ADC_CHANNEL_8;
	sConfigADC.Rank = 1;
	sConfigADC.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	sConfigADC.Offset = 0;
    
    HAL_ADC_Init(&hadc1);
    HAL_ADC_ConfigChannel(&hadc1,&sConfigADC);
}


float pendulum_read_voltage(void){

   uint16_t result = 0;
   float value;

    /* TODO: Start ADC */
        HAL_ADC_Start(&hadc1);
 /* TODO: Poll for conversion. Use timeout of 0xFF. */
        HAL_ADC_PollForConversion(&hadc1, 0xFF);
 /* TODO: Get ADC value */
        result = HAL_ADC_GetValue(&hadc1);
 /* TODO: Stop ADC */
        HAL_ADC_Stop(&hadc1);
 /* TODO: Compute voltage from ADC reading. Hint: 2^12-1 = 4095 */
    value = (3.3/4095)*result;
 /* TODO: Return the computed voltage */
return value;
}