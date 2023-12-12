#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "uart.h"
#include "motor.h"
#include "controller.h"

static TIM_HandleTypeDef _htim2;
static TIM_HandleTypeDef _htim3;
static TIM_HandleTypeDef _htim4;
static TIM_HandleTypeDef _htim11;

static TIM_OC_InitTypeDef _sConfigPULSE;
#define TIMERPERIOD 100000        //1kHz signal
#define BASECLKFREQ  100000000
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int32_t enc1_count=0;
static int32_t enc2_count=0;
static int32_t enc3_count=0;
static int32_t enc1_countold = 0;
static int32_t enc2_countold = 0;
static int32_t enc3_countold = 0;

float rev1 = 0;
float rev2 = 0;
float rev3 = 0;


static int32_t ele_enc_count=0;
static int32_t azi_enc_count=0;
static float ele_dutycycle;
static float azi_dutycycle;
static uint8_t _is_init = 0;
static float dutycycle1;
static float dutycycle2;
static float dutycycle3;

static float ele_duty;
static float azi_duty;

//SERVO
float offset =0;
float value;
float duty;


// pin variables for the launhcer encoders
int32_t PC10 = 0;
int32_t PC11 = 0;
int32_t PC10old = 0;
int32_t PC11old = 0;

int32_t PC0 = 0;
int32_t PA0 = 0;
int32_t PC0old = 0;
int32_t PA0old = 0;

int32_t PC8 = 0;
int32_t PC9 = 0;
int32_t PC8old = 0;
int32_t PC9old = 0;



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
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM11_CLK_ENABLE();
    // __HAL_RCC_TIM8_CLK_ENABLE();
 /*  Enable GPIOA clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

// PWM INITIALISATION
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_6;       //motor 1
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_10;       //motor 2
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_7;       //motor 3
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_7;       //elevation
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_6;       //Azimuth
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_9;       //SERVO
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM11;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

// DIR PIN INIT / PHase pin init    for launchers
	GPIO_InitTypeDef GPIO_InitStruct10;
	GPIO_InitStruct10.Pin = GPIO_PIN_15;		//PA15
	GPIO_InitStruct10.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct10.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct10);

    GPIO_InitTypeDef GPIO_InitStruct11;
	GPIO_InitStruct11.Pin = GPIO_PIN_13|GPIO_PIN_14;		//PB13 for azimuth, PB14 for elevation
	GPIO_InitStruct11.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct11.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct11);
        

 /*  Initialise timer 3 
*/

    _sConfigPULSE.OCMode = TIM_OCMODE_PWM1;
    _sConfigPULSE.Pulse = 0; // delays the toggle relative to timer
    _sConfigPULSE.OCPolarity = TIM_OCPOLARITY_HIGH;
    _sConfigPULSE.OCFastMode = TIM_OCFAST_DISABLE;


    _htim3.Instance = TIM3;
    _htim3.Init.Prescaler = 2; 
    _htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    _htim3.Init.Period = TIMERPERIOD/2;
    _htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;;

    HAL_TIM_PWM_Init(&_htim3);
    HAL_TIM_PWM_ConfigChannel(&_htim3, &_sConfigPULSE, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&_htim3, &_sConfigPULSE, TIM_CHANNEL_2);

    _htim2.Instance = TIM2;
    _htim2.Init.Prescaler = 2; 
    _htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    _htim2.Init.Period = TIMERPERIOD/2;
    _htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    HAL_TIM_PWM_Init(&_htim2);
    HAL_TIM_PWM_ConfigChannel(&_htim2, &_sConfigPULSE, TIM_CHANNEL_3);

    _htim4.Instance = TIM4;
    _htim4.Init.Prescaler = 2; 
    _htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    _htim4.Init.Period = TIMERPERIOD/2;
    _htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    HAL_TIM_PWM_Init(&_htim4);
    HAL_TIM_PWM_ConfigChannel(&_htim4, &_sConfigPULSE, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&_htim4, &_sConfigPULSE, TIM_CHANNEL_2);

    //SERVO
    _htim11.Instance = TIM11;

    _htim11.Init.Prescaler = 40;
    _htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
    _htim11.Init.Period = TIMERPERIOD/2;      //should equate to 1kHz
    _htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&_htim11);
    HAL_TIM_PWM_ConfigChannel(&_htim11, &_sConfigPULSE, TIM_CHANNEL_1);



    /* Set initial Timer 3, channel 1 compare value */
    /* Start Timer 3, channel 1 */
   HAL_TIM_PWM_Start(&_htim3, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&_htim3, TIM_CHANNEL_2);
   HAL_TIM_PWM_Start(&_htim2, TIM_CHANNEL_3);
   HAL_TIM_PWM_Start(&_htim4, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&_htim4, TIM_CHANNEL_2);
   HAL_TIM_PWM_Start(&_htim11, TIM_CHANNEL_1);
//    HAL_TIM_PWM_Start(&_htim8, TIM_CHANNEL_2);
 }



void velocity_adjust(float thrust1,float thrust2,float thrust3)
{
    // thrust1 = 10000;
    // thrust2 = 10000;
    // thrust3 = 10000;

    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);		//output of dire pin
	// dutycycle = (TIMERPERIOD)*thrust;	
    dutycycle1 = thrust1;	
	__HAL_TIM_SET_COMPARE(&_htim3, TIM_CHANNEL_1, (uint32_t)dutycycle1);

    // printf("set velocity %f\n", dutycycle);
    dutycycle2 = thrust2;	    //PB10
	__HAL_TIM_SET_COMPARE(&_htim2, TIM_CHANNEL_3, (uint32_t)dutycycle2);
    dutycycle3 = thrust3;	    //pc7
	__HAL_TIM_SET_COMPARE(&_htim4, TIM_CHANNEL_2, (uint32_t)dutycycle3);
    // printf("Duty %0.1f\n", dutycycle1);
}


void elevation_adjust(float lift)
{
    // ele_duty = lift;
    if (lift < 0){
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
        ele_dutycycle = fabs(lift) + 5000;     //adjust for direction. Offset just reduces the dead zone slightly
    } else
        if (lift > 0) {
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);		//output of dire pin
            ele_dutycycle = lift + 5000;	
        }
        else{
            ele_dutycycle = 0;
        }
   // dutycycle = (TIMERPERIOD)*thrust;	
    // printf("duty cycle %0.2f\n", ele_dutycycle);
	__HAL_TIM_SET_COMPARE(&_htim3, TIM_CHANNEL_2, (uint32_t)ele_dutycycle);
}

void azimuth_adjust(float aim)
{
    if (aim < 0){
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
        azi_dutycycle = fabs(aim);     //adjust for direction
    } else
        if (aim > 0) {
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);		//output of dire pin
            azi_dutycycle = aim;	
        }
        else{
            azi_dutycycle = 0;
        }
   // dutycycle = (TIMERPERIOD)*thrust;	
    // printf("duty cycle %0.2f\n", ele_dutycycle);
	__HAL_TIM_SET_COMPARE(&_htim4, TIM_CHANNEL_1, (uint32_t)azi_dutycycle);
}

void servo_adjust(float deg){

    value = (deg/180);
    offset = 2000;                  // approximated
    duty = 4000*value + offset;     //This 4000 isn't exactly right. Couldn't get the PWM to be exact for some reason
    // duty = deg;
    //10000 duty cycle is 4ms pulse. Therefore 2500 is 1ms. Offset has been adjusted to give around 180deg of motion
	__HAL_TIM_SET_COMPARE(&_htim11, TIM_CHANNEL_1, (uint32_t)duty);
}



 void motor_encoder_init(void){
//  OLD CODE
/* TODO: Enable GPIOC clock */
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        // __HAL_RCC_GPIOH_CLK_ENABLE();
//  /* TODO: Initialise PC0, PC1 with:
//  - Pin 0|1
//  - Interrupt rising and falling edge
//  - No pull
//  - High frequency */

        // LAUNCHER MOTORS
        GPIO_InitTypeDef GPIO_EncInitStruct;
        GPIO_EncInitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;       //PC10, PC11
        GPIO_EncInitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
        GPIO_EncInitStruct.Pull = GPIO_NOPULL;
        GPIO_EncInitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOC, &GPIO_EncInitStruct);

        GPIO_InitTypeDef GPIO_EncInitStruct2;         
        GPIO_EncInitStruct2.Pin = GPIO_PIN_0;                  //PC0, PA0
        GPIO_EncInitStruct2.Mode = GPIO_MODE_IT_RISING_FALLING;
        GPIO_EncInitStruct2.Pull = GPIO_NOPULL;
        GPIO_EncInitStruct2.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOC, &GPIO_EncInitStruct2);    
        HAL_GPIO_Init(GPIOA, &GPIO_EncInitStruct2);

        GPIO_InitTypeDef GPIO_EncInitStruct3;
        GPIO_EncInitStruct3.Pin = GPIO_PIN_8|GPIO_PIN_9;       //PC8, PC9
        GPIO_EncInitStruct3.Mode = GPIO_MODE_IT_RISING_FALLING;
        GPIO_EncInitStruct3.Pull = GPIO_NOPULL;
        GPIO_EncInitStruct3.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOC, &GPIO_EncInitStruct3);

        //ELEVATION

        GPIO_InitTypeDef GPIO_EncInitStruct4;
        GPIO_EncInitStruct4.Pin = GPIO_PIN_2|GPIO_PIN_3;        //PC2, PC3
        GPIO_EncInitStruct4.Mode = GPIO_MODE_IT_RISING_FALLING;
        GPIO_EncInitStruct4.Pull = GPIO_NOPULL;
        GPIO_EncInitStruct4.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOC, &GPIO_EncInitStruct4);

        //AZIMUTH

        GPIO_InitTypeDef GPIO_EncInitStruct5;
        GPIO_EncInitStruct5.Pin = GPIO_PIN_1;
        GPIO_EncInitStruct5.Mode = GPIO_MODE_IT_RISING_FALLING;     //PB1,Pc4
        GPIO_EncInitStruct5.Pull = GPIO_NOPULL;
        GPIO_EncInitStruct5.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_EncInitStruct5);
        GPIO_EncInitStruct5.Pin = GPIO_PIN_4;
        HAL_GPIO_Init(GPIOC, &GPIO_EncInitStruct5);

 
 /* TODO: Set priority of external interrupt lines 0,1 to 0x0f, 0x0f
 To find the IRQn_Type definition see "MCHA3500 Windows Toolchain\workspace\STM32Cube_F4_FW\Drivers\
CMSIS\Device\ST\STM32F4xx\Include\stm32f446xx.h" */


// CHANGE THESE BACK TO 0 and 1
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0x0f, 0x0f);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0x0f, 0x0f);
    HAL_NVIC_SetPriority(EXTI2_IRQn, 0x0f, 0x0f);
    HAL_NVIC_SetPriority(EXTI3_IRQn, 0x0f, 0x0f);
    HAL_NVIC_SetPriority(EXTI4_IRQn, 0x0f, 0x0f);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0x0f, 0x0f);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0x0f, 0x0f);

 /* TODO: Enable external interrupt for lines 0, 1 */
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

 }


void EXTI0_IRQHandler(void)     //First launcher
 {

    PC0 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
    PA0 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

    if (PC0 != PC0old)
        {
        enc2_count++;
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
        }
    if (PA0 != PA0old)
        {
        enc2_count++;
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
        }
    
    PC0old = PC0;
    PA0old = PA0;
 /* TODO: Reset interrupt */
    // HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);

}

void EXTI9_5_IRQHandler(void)     //First launcher
 {

    PC8 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);
    PC9 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9);

    if (PC8 != PC8old)
        {
        enc3_count++;
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
        }
    if (PC9 != PC9old)
        {
        enc3_count++;
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
        }
    
    PC8old = PC8;
    PC9old = PC9;
    printf("intt\n");//
 /* TODO: Reset interrupt */
    // HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);

}


void EXTI15_10_IRQHandler(void)     //First launcher
 { 
    PC10 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10);
    PC11 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11);

    if (PC10 != PC10old)
        {
        enc1_count++;
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
        }
    if (PC11 != PC11old)
        {
    enc1_count++;
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
        }
    
    PC10old = PC10;    
    PC11old = PC11;
    
}

///////////////////////////// HANDLERS IN CHARGE OF AZIMUTH///////////////////////
void EXTI1_IRQHandler(void) {       
/* TODO: Check if PC0 == PC1. Adjust encoder count accordingly. */
    if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)) == (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4)))
        {
        azi_enc_count++;
        }
    else{
        azi_enc_count--;
        }

    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
    }

void EXTI4_IRQHandler(void) {
/* TODO: Check if PC0 == PC1. Adjust encoder count accordingly. */
    if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)) == (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4)))
        {
        azi_enc_count--;
        } 
    else{
        azi_enc_count++;
        }
 /* TODO: Reset interrupt */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
 }


////////////////////////////// HANDLERS IN CHARGE OF ELEVATION //////////////
void EXTI2_IRQHandler(void) {       
    if ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)) == (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)))
        {
        ele_enc_count--;
        }
    else{
        ele_enc_count++;
        }

    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
    }

void EXTI3_IRQHandler(void) {
    if ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)) == (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)))
        {
        ele_enc_count++;
        } 
    else{
        ele_enc_count--;
        }
 /* TODO: Reset interrupt */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
 }



 int32_t motor_encoder1_getValue(void)
    {
        // return enc1_count,enc2_count,enc3_count;
        return enc1_count;
    }
 int32_t motor_encoder2_getValue(void)
    {
        return enc2_count;
    }
 int32_t motor_encoder3_getValue(void)
    {
        return enc3_count;
    }

int32_t ele_encoder_getValue(void)
    {
        return ele_enc_count;
    }

void ele_encoder_reset(void)
    {   ele_enc_count = 0;
        azi_enc_count = 0;
        control_set_elevation(0);
        control_set_azimuth(0);
        printf("Elevation and Azimuth Calibrated\n");
        return;
    }

int32_t azi_encoder_getValue(void)
    {
        return azi_enc_count;
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

        float angle = 60*cnt/(12*4.995);          //convert encoder count to revs/min
        return angle;

    }

float  elevation_encoder_getAngle(int32_t elcnt)
    {
        // 14 cpr * gear ratio of 298, however this is for only 1 encoder pion.
        //2*14*298 cpr.
        // gear ratio of gears is approximated with diameters on creo model.
        float angle = (31.25/50)*360*elcnt/(14*298*2);          //encoder count in degrees. Check this stuff at the bottom
        return angle;
    }
