#ifndef DUMMY_TASK_H
#define DUMMY_TASK_H

#include <stdint.h>

void    dummy_task_init(void);
void    dummy_task_deinit(void);
void    dummy_task_start(void);
void    dummy_task_stop(void);
uint8_t dummy_task_is_running(void);

#endif
