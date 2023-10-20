#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "heartbeat_task.h"

#define HEARTBEATPERIOD 1000

static void _heartbeat_update(void *arg);

static osThreadId_t _heartbeatThreadID;
static osThreadAttr_t _heartbeatThreadAttr = 
{
    .name = "heartbeat",
    .priority = osPriorityIdle,
    .stack_size = 128
};

static uint8_t _is_running = 0;
static uint8_t _is_init = 0;

void heartbeat_task_init(void)
{
    if (!_is_init)
    {
        /* Configure PA5 in output pushpull mode */
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitTypeDef  GPIO_InitStructure;
        GPIO_InitStructure.Pin = GPIO_PIN_5;
        GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStructure.Pull = GPIO_PULLUP;
        GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

        // CMSIS-RTOS API v2 Timer Documentation: https://www.keil.com/pack/doc/CMSIS/RTOS2/html/group__CMSIS__RTOS__TimerMgmt.html
        _heartbeatThreadID = osThreadNew(_heartbeat_update, NULL, &_heartbeatThreadAttr);   // Create the thread in the OS scheduler. 
        // Note: The thread starts automatically when osThreadNew is called
        _is_running = 1;
        _is_init = 1;
    }
}

void heartbeat_task_start(void)
{
    if (!_is_running)
    {
        osThreadResume(_heartbeatThreadID);
        _is_running = 1;
    }
}

void heartbeat_task_stop(void)
{
    if (_is_running)
    {
        osThreadSuspend(_heartbeatThreadID);
        _is_running = 0;
    }
}

uint8_t heartbeat_task_is_running(void)
{
    return _is_running;
}

void _heartbeat_update(void *arg)
{
    UNUSED(arg); 
    while(1)
    {
        // Toggle the heartbeat LED
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        // Non-blocking delay to wait
        osDelay(HEARTBEATPERIOD);
    }
}

void heartbeat_task_deinit(void)
{
    _is_init = 0;
    _is_running = 0;
}