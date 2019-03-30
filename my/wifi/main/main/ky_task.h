#ifndef __KY_TASK_H
#define __KY_TASK_H

#include <stdint.h>

void ky_create_task(void);

void led_task(void);
void key_test(void);
void key_task(void);
void ledc_task(void);



#endif
