#ifndef __KEY_H
#define __KEY_H

#include <stdint.h>

typedef enum  {
    KEY_NONE_PRESS = 0,
    KEY_SINGLE_CLICK,
    KEY_DOUBLE_CLICK,
    KEY_LONG_RRESS_START,
    KEY_LONG_PRESS_HOLD
}key_event_t;

void key_init(void);
key_event_t get_key_event(uint8_t key_num);
void key_ticks(void);

#endif
