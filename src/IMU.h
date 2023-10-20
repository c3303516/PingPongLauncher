#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void IMU_init(void);
void IMU_read(void);
float get_accY(void);
float get_accZ(void);
float get_accX(void);
float get_gyroY(void);
double get_acc_angle(void);

#endif