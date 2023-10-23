#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "uart.h"
#include "data_logging.h"
#include "pendulum.h"
#include "IMU.h"
#include "motor.h"

/* Variable declarations */
 static void (*log_function)(void);
 uint16_t logCount = 0;
 float result = 0;

/* Function declarations */
static void log_pendulum(void *argument);
static void log_pointer(void *argument);
static void log_imu(void);

static osTimerId_t _data_logger_timer_id;
static osTimerAttr_t _data_logger_timer_attr =
{
    .name = "dataLoggerTimer"
};


 /* Function defintions */
 static void log_pendulum(void *argument)
 {  
    float sampleTime;
 /* TODO: Supress compiler warnings for unused arguments */
    UNUSED(argument);
 /* TODO: Read the potentiometer voltage */
        //result = pendulum_read_voltage();

        //hijack this for encoder use
        result = motor_encoder_getValue();
 /* TODO: Print the sample time and potentiometer voltage to the serial terminal in the format [time],[
voltage] */
    sampleTime = logCount*0.005;
    printf("%0.3f,  %0.3f \n", sampleTime, result);
 /* TODO: Increment log count */
       logCount++;
 /* TODO: Stop logging once 5 seconds is reached (Complete this once you have created the stop function
in the next step) */
    if (logCount >= 1000){
        logging_stop();
        printf("finished\n");
    }
 }

static void log_imu(void)
{
    // IMU_read();
    // float angle = get_acc_angle();
    // float angvel = get_gyroY();     //want angle of rotation about Y axis
    // result = motor_encoder_getAngle();
    // float sampleTime = logCount*0.005;

    // printf("%f,%f,%f,%f\n", sampleTime, angle, angvel, result);
    // logCount++;

    //     if (logCount >= 1000){
    //     logging_stop();
    //     printf("finished\n");
    // }
}



void logging_init(void)  
 { 
     /* TODO: Initialise timer for use with pendulum data logging */
     _data_logger_timer_id = osTimerNew(log_pointer, osTimerPeriodic, NULL, &_data_logger_timer_attr);
 }

static void log_pointer(void *argument)
{
    UNUSED(argument);
 /* Call function pointed to by log_function */
    (*log_function)();
}


void pend_logging_start(void)
 {
      /* TODO: Change function pointer to the pendulum logging function */
     log_function = &log_pendulum;
     /* TODO: Reset the log counter */
    logCount = 0;
    /* TODO: Start data logging timer at 200Hz */
     osTimerStart(_data_logger_timer_id, 5);
 }


void logging_stop(void)
{
    /* TODO: Stop data logging timer */
    osTimerStop(_data_logger_timer_id);
}


void imu_logging_start(void)
{
    log_function = &log_imu;
    logCount = 0;
    osTimerStart(_data_logger_timer_id, 5);
}