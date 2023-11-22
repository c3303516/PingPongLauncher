#ifndef MOTOR_H
#define MOTOR_H

/* Add function prototypes here */

 void motor_PWM_init(void);
 void motor_encoder_init(void);
 void EXTI1_IRQHandler(void);
 void EXTI2_IRQHandler(void);
 void EXTI3_IRQHandler(void);
 void EXTI4_IRQHandler(void);
 void EXTI0_IRQHandler(void);
 void EXTI9_5_IRQHandler(void);
 void EXTI15_10_IRQHandler(void);
 int32_t motor_encoder1_getValue(void);
 int32_t motor_encoder2_getValue(void);
 int32_t motor_encoder3_getValue(void);
 float motor_encoder_getAngle(int32_t);
 float motor_encoder_getRev(int32_t);
//  void direction_adjust(float);
//  int microstep(float v);
// void comms_loop_init(void)
void velocity_adjust(float,float,float);
void elevation_adjust(float);
void azimuth_adjust(float);
float elevation_encoder_getAngle(int32_t);
int32_t ele_encoder_getValue(void);
int32_t azi_encoder_getValue(void);

#endif