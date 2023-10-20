
#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "uart.h"
#include "tm_stm32_mpu6050.h"
#include "IMU.h"
#include "math.h"
// #ifndef M_PI
// #define M_PI 3.14159265358979323846
// #endif
/* Variable declarations */
TM_MPU6050_t IMU_datastruct;

/* Function defintions */

void IMU_read(void)
{
    TM_MPU6050_ReadAll(&IMU_datastruct);
}

float get_accY(void)
{
    float acc;
    if (IMU_datastruct.Accelerometer_Y<1)
        {   
            acc = IMU_datastruct.Accelerometer_Y;
            acc = 4*9.81*acc/32786;
        }
    if (IMU_datastruct.Accelerometer_Y>1)
        {
            acc = IMU_datastruct.Accelerometer_Y;
            acc = 4*9.81*acc/32767;
        }
    return(acc);
}

float get_accZ(void)
{
    float acc;
    if (IMU_datastruct.Accelerometer_Z<1)
        {   
            acc = IMU_datastruct.Accelerometer_Z;
            acc = 4*9.81*acc/32786;
        }
    if (IMU_datastruct.Accelerometer_Z>1)
        {
            acc = IMU_datastruct.Accelerometer_Z;
            acc = 4*9.81*acc/32767;
        }
    return(acc);
}
float get_accX(void)
{
    float acc;
    if (IMU_datastruct.Accelerometer_X<1)
        {   
            acc = IMU_datastruct.Accelerometer_X;
            acc = 4*9.81*acc/32786;
        }
    if (IMU_datastruct.Accelerometer_X>1)
        {
            acc = IMU_datastruct.Accelerometer_X;
            acc = 4*9.81*acc/32767;
        }
    return(acc);
}

float get_gyroY(void)
{
        float acc;
    if (IMU_datastruct.Accelerometer_Y<1)
        {   
            acc = IMU_datastruct.Gyroscope_Y;
            acc = (250*M_PI/180)*acc/32786;
        }
    if (IMU_datastruct.Accelerometer_Y>1)
        {
            acc = IMU_datastruct.Gyroscope_Y;
            acc = (250*M_PI/180)*acc/32767;
        }
    return(acc);
}

double get_acc_angle(void)
{
    double angle;
    angle = atan2(get_accZ(),get_accX());      //changed to Y to X for rotation about Y axis, also removed the minus
    return angle;
}


void IMU_init(void)
{
/* TODO: Initialise IMU with AD0 LOW, accelleration sensitivity +-4g, gyroscope +-250 deg/s */
    
    //TM_MPU6050_SetAccelerometer(IMU_datastruct, 0x01);
    //TM_MPU6050_SetGyroscope (IMU_datastruct, 0x00);
    
    TM_MPU6050_Init(&IMU_datastruct, TM_MPU6050_Device_0, TM_MPU6050_Accelerometer_4G, TM_MPU6050_Gyroscope_250s);
}