#ifndef MOTOR_H
#define MOTOR_H

/* Add function prototypes here */

 void motor_PWM_init(void);
 void motor_encoder_init(void);
//  void EXTI0_IRQHandler(void);
//  void EXTI1_IRQHandler(void);
 int32_t motor_encoder_getValue(void);
 float motor_encoder_getAngle(void);
 void direction_adjust(float);
 int microstep(float v);
void velocity_adjust(float);

#endif