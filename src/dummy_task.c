#include "dummy_task.h"

#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "uart.h"
#include "pendulum.h"
#include "motor.h"
#include "IMU.h"
#include "kalman.h"
#include "controller.h"
#include "control_loop.h"

static void dummy_task_update(void *arg);

static osThreadId_t _dummyTaskThreadID;
static osThreadAttr_t _dummyTaskThreadAttr = 
{
    .name = "heartbeat",
    .priority = osPriorityIdle,
    .stack_size = 128
};

static uint8_t _is_running = 0;
static uint8_t _is_init = 0;
static uint8_t mode = 0;
// static float thrustpercent;



void dummy_task_init(void)
{
    if (!_is_init)
    {
        // CMSIS-RTOS API v2 Timer Documentation: https://www.keil.com/pack/doc/CMSIS/RTOS2/html/group__CMSIS__RTOS__TimerMgmt.html
        _dummyTaskThreadID = osThreadNew(dummy_task_update, NULL, &_dummyTaskThreadAttr);   // Create the thread in the OS scheduler. 
        // Note: The thread starts automatically when osThreadNew is called
        _is_running = 1;
        _is_init = 1;

        __HAL_RCC_GPIOC_CLK_ENABLE();
        GPIO_InitTypeDef GPIO_InitStruct4;
        GPIO_InitStruct4.Pin = GPIO_PIN_5;       //init pin 5
        GPIO_InitStruct4.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct4.Pull = GPIO_PULLDOWN;
        //GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct4);

    }
}

void dummy_task_start(void)
{
    if (!_is_running)
    {
        osThreadResume(_dummyTaskThreadID);
        _is_running = 1;
    }
}

void dummy_task_stop(void)
{
    if (_is_running)
    {
        osThreadSuspend(_dummyTaskThreadID);
        _is_running = 0;
    }
}

uint8_t dummy_task_is_running(void)
{
    return _is_running;
}

void dummy_task_update(void *arg)
{
    while(1)
    {   
        // mode = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_5);      //read switch
        // // kalman_timer_start();                           //start kalman for calibration
        // osDelay(2000);                                  //delay
        // printf("mode = %d\n",mode);
        // // kalman_timer_stop();                            //stop kalman
        printf("Dummy\n");
        control_timer_start();                          //start control
        // osDelay(1000);                                  //balance for a second
        // if (mode == 1){                                 //if switch is closed, change yref
        // control_set_speed(12*M_PI);                     //set speed to high velocity
        // osDelay(8000);                                  //approx time for 20m dash
        // // control_set_speed(0);                           //set speed to zero
        // }

        // thrustpercent = 0;
        // velocity_adjust(thrustpercent);

        // osDelay(2000);
        // thrustpercent = 10;
        // velocity_adjust(thrustpercent);
        // osDelay(2000);
        aim_timer_start();
        
        dummy_task_stop();
    }
}

void dummy_task_deinit(void)
{
    _is_init = 0;
    _is_running = 0;
}
