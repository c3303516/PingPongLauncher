#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "uart.h"
#include "motor.h"

static TIM_HandleTypeDef _htim3;
static TIM_OC_InitTypeDef _sConfigPULSE;
#define TIMERPERIOD 100         //1MHz signal
#define BASECLKFREQ  100000000
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int32_t enc_count;
static int32_t result;
static uint8_t _is_init = 0;
static int32_t dir;
static int micro = 1;
static float prescale;

static void module_comms_report_timer_exp(void *arg);
osTimerId_t     _comms_report_timer_id;  
osTimerAttr_t   _comms_report_timer_attr = 
{
    .name = "commsReportTimer"
};

 void motor_PWM_init(void)
 {
 /*  Enable TIM3 clock */
    __HAL_RCC_TIM3_CLK_ENABLE();
 /*  Enable GPIOA clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

// PWM INITIALISATION
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

// DIR PIN INIT
        GPIO_InitTypeDef GPIO_InitStruct2;
        GPIO_InitStruct2.Pin = GPIO_PIN_6|GPIO_PIN_8;
        GPIO_InitStruct2.Mode = GPIO_MODE_OUTPUT_PP;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct2);
// MICROSTEPPER INIT
    GPIO_InitTypeDef GPIO_InitStruct3;
    GPIO_InitStruct3.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct3.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct3);

 /*  Initialise timer 3 
*/
    _htim3.Instance = TIM3;
    _htim3.Init.Prescaler = 1; 
    _htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    _htim3.Init.Period = TIMERPERIOD;
    _htim3.Init.ClockDivision = 1;


    _sConfigPULSE.OCMode = TIM_OCMODE_PWM1;
    _sConfigPULSE.Pulse = 0; // delays the toggle relative to timer
    _sConfigPULSE.OCPolarity = TIM_OCPOLARITY_HIGH;
    _sConfigPULSE.OCFastMode = TIM_OCFAST_DISABLE;

      HAL_TIM_PWM_Init(&_htim3);
      HAL_TIM_PWM_ConfigChannel(&_htim3, &_sConfigPULSE, TIM_CHANNEL_1);
      HAL_TIM_PWM_ConfigChannel(&_htim3, &_sConfigPULSE, TIM_CHANNEL_2);

    /* Set initial Timer 3, channel 1 compare value */
    /* Start Timer 3, channel 1 */
   HAL_TIM_PWM_Start(&_htim3, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&_htim3, TIM_CHANNEL_2);
      static float ang_velocity = 2*M_PI/200; // in rad/s    //1 step/s
      velocity_adjust(ang_velocity);
 }

void direction_adjust(float vel)
{   int i = 0;
    __HAL_TIM_SET_COMPARE(&_htim3, TIM_CHANNEL_1, TIMERPERIOD/2);  //these 'turn on' velocity
    __HAL_TIM_SET_COMPARE(&_htim3, TIM_CHANNEL_2, TIMERPERIOD/2);
    if (vel > 0.05){
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
    i = 1;
    }
    else{
     if (vel < -0.05){
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
        i = 2;
        }
        else{
        __HAL_TIM_SET_COMPARE(&_htim3, TIM_CHANNEL_1, 0);  
        __HAL_TIM_SET_COMPARE(&_htim3, TIM_CHANNEL_2, 0);
        i = 3;
        }
    }
    //printf("dir = %0.1d\n", i);
}

int microstep(float v)
{
    if (v>4*M_PI)
    {
    //no microstp
    micro = 1;
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);    //M0
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);    //M1
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);    //M2
    }
    else{ 
        if (v > 2*M_PI)
        {
        //half step
        micro = 2;
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);    //M0
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);    //M1
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);    //M2
        }
        else{ 
            if (v > M_PI)
            {
            //quarter step
            micro = 4;
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);    //M0
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);    //M1
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);    //M2
            }
            else{ 
                if (v > M_PI/2)
                { 
                micro = 8;
                HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);    //M0
                HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);    //M1
                HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);    //M2
                }
                else{ 
                    if (v > M_PI/4)
                    { 
                    micro = 16;
                    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);    //M0
                    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);    //M1
                    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);    //M2
                    }   
                    else{
                        micro = 32;
                        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);    //M0
                        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);    //M1
                        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);    //M2
                        }
                    }
                }
            }
        }

    return micro;

}

void velocity_adjust(float velocity)
{
    direction_adjust(velocity);
    //try to keep 200Hz to motors
    velocity = fabs(velocity);                  //abs value
    micro = microstep(velocity);                //initialise microstpping
    //printf("%d micro \n", micro);
   // micro = 4;
    int stepsperrev = 200*micro;
   double frequency = velocity*stepsperrev/(2*M_PI);

    // bse clk/period = 1000kHz
   prescale = (BASECLKFREQ/TIMERPERIOD)*(1/frequency);
   //printf("\n%0.4f prescale \n", prescale);
   __HAL_TIM_SET_PRESCALER(&_htim3, prescale);

}


 void motor_encoder_init(void){

/* TODO: Enable GPIOC clock */
        __HAL_RCC_GPIOB_CLK_ENABLE();
 /* TODO: Initialise Pb1, PB2 with:
 - Pin 0|1
 - Interrupt rising and falling edge
 - No pull
 - High frequency */

        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;       //init pin 0
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


 /* TODO: Set priority of external interrupt lines 0,1 to 0x0f, 0x0f
 To find the IRQn_Type definition see "MCHA3500 Windows Toolchain\workspace\STM32Cube_F4_FW\Drivers\
CMSIS\Device\ST\STM32F4xx\Include\stm32f446xx.h" */

    HAL_NVIC_SetPriority(EXTI1_IRQn, 0x0f, 0x0f);
    HAL_NVIC_SetPriority(EXTI2_IRQn, 0x0f, 0x0f);

 /* TODO: Enable external interrupt for lines 0, 1 */
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);

 }

void EXTI1_IRQHandler(void)
 {
/* TODO: Check if PC0 == PC1. Adjust encoder count accordingly. */
    if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)) == (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)))
        {
        enc_count++;
        }
    else{
        enc_count--;
        }

    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);

}


void EXTI2_IRQHandler(void) {

/* TODO: Check if PC0 == PC1. Adjust encoder count accordingly. */


    if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)) == (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)))
        {
        enc_count--;
        } 
    else{
        enc_count++;
        }
 /* TODO: Reset interrupt */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
 }

 int32_t motor_encoder_getValue(void)
    {
        return enc_count;
    }

float motor_encoder_getAngle(void)
    {
        int32_t cnt = motor_encoder_getValue();
        float angle = 2*M_PI*cnt/(4*1024);          //convert encoder count to radians
        return angle;
    }

