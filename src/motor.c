#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "uart.h"
#include "motor.h"

static TIM_HandleTypeDef _htim3;
static TIM_OC_InitTypeDef _sConfigPULSE;
#define TIMERPERIOD 100000        //1kHz signal
#define BASECLKFREQ  100000000
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int32_t enc1_count;
static int32_t enc2_count;
static int32_t enc3_count;
static int32_t ele_enc_count;
static float ele_dutycycle;
static uint8_t _is_init = 0;
static float dutycycle;

static float ele_duty;

int32_t PB3 = 0;
int32_t PC3 = 0;
int32_t PB3old = 0;
int32_t PC3old = 0;

int32_t PB12 = 0;
int32_t PA12 = 0;
int32_t PB12old = 0;
int32_t PA12old = 0;


int32_t PA5 = 0;
int32_t PC5 = 0;
int32_t PA5old = 0;
int32_t PC5old = 0;



static void module_comms_report_timer_exp(void *arg);
osTimerId_t     _comms_report_timer_id;  
osTimerAttr_t   _comms_report_timer_attr = 
{
    .name = "commsReportTimer"
};

// Time to update velocity and reference reports
// void comms_loop_init(void)  
//  { 
//      /* TODO: Initialise timer for use with pendulum data logging */
//      _comms_report_timer_id = osTimerNew(comms_update, osTimerPeriodic, NULL, & _comms_report_timer_attr);
//     //  _kalman_timer_id = osTimerNew(kalman_loop_update, osTimerPeriodic, NULL, &_kalman_timer_attr);
//  }

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
    GPIO_InitStruct.Pin = GPIO_PIN_4;       //motor 1
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitTypeDef GPIO_InitStruct3;
    GPIO_InitStruct3.Pin = GPIO_PIN_7;       //elevation
    GPIO_InitStruct3.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct3.Pull = GPIO_NOPULL;
    GPIO_InitStruct3.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct3.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct3);

// DIR PIN INIT / PHase pin init    for launchers
	GPIO_InitTypeDef GPIO_InitStruct2;
	GPIO_InitStruct2.Pin = GPIO_PIN_5;		//PB5
	GPIO_InitStruct2.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct2.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct2);

    GPIO_InitTypeDef GPIO_InitStruct5;
	GPIO_InitStruct5.Pin = GPIO_PIN_14;		//PB14 for elevation
	GPIO_InitStruct5.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct5.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct5);
        

 /*  Initialise timer 3 
*/
    _htim3.Instance = TIM3;
    _htim3.Init.Prescaler = 2; 
    _htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    _htim3.Init.Period = TIMERPERIOD/2;
    _htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;;


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
 }



void velocity_adjust(float thrust)
{
    //try to keep 200Hz to motors
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);		//output of dire pin
	// dutycycle = (TIMERPERIOD)*thrust;	
    dutycycle = thrust;	
	__HAL_TIM_SET_COMPARE(&_htim3, TIM_CHANNEL_1, (uint32_t)dutycycle);

    // printf("set velocity %f\n", dutycycle);
}


void elevation_adjust(float lift)
{
    // ele_duty = lift;
    if (lift < 0){
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
        ele_dutycycle = fabs(lift);     //adjust for direction
    } else
        if (lift > 0) {
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);		//output of dire pin
            ele_dutycycle = lift;	
        }
        else{
            ele_dutycycle = 0;
        }
   // dutycycle = (TIMERPERIOD)*thrust;	
    printf("duty cycle %0.2f\n", ele_dutycycle);
	__HAL_TIM_SET_COMPARE(&_htim3, TIM_CHANNEL_2, (uint32_t)ele_dutycycle);
}




 void motor_encoder_init(void){
//  OLD CODE
/* TODO: Enable GPIOC clock */
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
//  /* TODO: Initialise PC0, PC1 with:
//  - Pin 0|1
//  - Interrupt rising and falling edge
//  - No pull
//  - High frequency */

        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;       //init pin 0,1
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitTypeDef GPIO_InitStruct;           //encoder 1
        GPIO_InitStruct.Pin = GPIO_PIN_12;       //init pin 12
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


        GPIO_InitTypeDef GPIO_InitStruct2;       //encoder 3
        GPIO_InitStruct2.Pin = GPIO_PIN_3;       //init pin 3
        GPIO_InitStruct2.Mode = GPIO_MODE_IT_RISING_FALLING;
        GPIO_InitStruct2.Pull = GPIO_NOPULL;
        GPIO_InitStruct2.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct2);        //PB3
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct2);        //PC3
 /* TODO: Set priority of external interrupt lines 0,1 to 0x0f, 0x0f
 To find the IRQn_Type definition see "MCHA3500 Windows Toolchain\workspace\STM32Cube_F4_FW\Drivers\
CMSIS\Device\ST\STM32F4xx\Include\stm32f446xx.h" */


// CHANGE THESE BACK TO 0 and 1
    HAL_NVIC_SetPriority(EXTI3_IRQn, 0x0f, 0x0f);
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0x0f, 0x0f);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0x0f, 0x0f);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0x0f, 0x0f);

 /* TODO: Enable external interrupt for lines 0, 1 */
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

 }

void EXTI3_IRQHandler(void)     //First launcher
 {

    PB3 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
    PC3 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);

    if (PB3 != PB3old)
        {
        enc3_count++;
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
        }
    if (PC3 != PC3old)
        {
        enc3_count++;
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
        }
    
    PB3old = PB3;
    PC3old = PC3;
 /* TODO: Reset interrupt */
    // HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);

}

void EXTI5_IRQHandler(void)     //First launcher
 {

    PA5 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
    PC5 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5);

    if (PA5 != PA5old)
        {
        enc2_count++;
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
        }
    if (PC5 != PC5old)
        {
        enc2_count++;
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
        }
    
    PA5old = PA5;
    PC5old = PC5;
 /* TODO: Reset interrupt */
    // HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);

}

void EXTI9_5_IRQHandler(void)     //First launcher
 {

    PA12 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);
    PB12 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);

    if (PA12 != PA12old)
        {
        enc1_count++;
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
        }
    if (PB12 != PB12old)
        {
        enc1_count++;
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
        }
    
    PA12old = PA12;    
    PB12old = PB12;
 /* TODO: Reset interrupt */
    // HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);

}


void EXTI0_IRQHandler(void) {       //attempting to link intterupst for elevation.
/* TODO: Check if PC0 == PC1. Adjust encoder count accordingly. */
    if ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)) == (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1)))
        {
        ele_enc_count++;
        }
    else{
        ele_enc_count--;
        }

    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
    }



void EXTI1_IRQHandler(void) {
/* TODO: Check if PC0 == PC1. Adjust encoder count accordingly. */
    if ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)) == (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1)))
        {
        ele_enc_count--;
        } 
    else{
        ele_enc_count++;
        }
 /* TODO: Reset interrupt */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
 }

 int32_t motor_encoder_getValue(void)
    {
        return enc_count;
    }

int32_t ele_encoder_getValue(void)
    {
        return ele_enc_count;
    }


float motor_encoder_getAngle(int32_t cnt)
    {
        // int32_t cnt = motor_encoder_getValue();

        // 12 * gear ratio, in this case 5.

        float angle = 2*M_PI*cnt/(12*4.995);          //convert encoder count to radians. Check this stuff at the bottom
        return angle;
    }

float motor_encoder_getRev(int32_t cnt)
    {
        // int32_t cnt = motor_encoder_getValue();

        // 12 * gear ratio, in this case 5.

        float angle = cnt/(12*4.995);          //convert encoder count to radians. Check this stuff at the bottom
        return angle;
    }

float  elevation_encoder_getAngle(int32_t elcnt)
    {
        // int32_t cnt = motor_encoder_getValue();

        // 14 cpr * gear ratio of 298.

        float angle = 360*elcnt/(14*298);          //encoder count in degrees. Check this stuff at the bottom
        return angle;
    }
