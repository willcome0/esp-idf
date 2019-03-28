#include "key.h"
#include "driver/gpio.h"
#include "multi_button.h"

#define KEY_NUM 5
struct Button key[KEY_NUM];

/******** key0 config ********/
#define KEY0_PIN_NUM    GPIO_NUM_21
static uint8_t read_key0_gpio(void)
{
    return gpio_get_level(KEY0_PIN_NUM);
}
static void KEY0_Handler_LONG_RRESS_START(void *btn)
{
    printf("长按\r\n");
}



void key_init(void)
{
    gpio_config_t io_conf;

    /******** key0 gpio init ********/
    io_conf.pin_bit_mask = 1 << KEY0_PIN_NUM;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    button_init(&key[0], read_key0_gpio, io_conf.pull_up_en == GPIO_PULLUP_ENABLE ? 0 : 1);
    button_attach(&key[0], LONG_RRESS_START, KEY0_Handler_LONG_RRESS_START); // long press
    button_start(&key[0]);
}

key_event_t get_key_event(uint8_t key_num)
{
    if (key_num>=KEY_NUM)
    {
        printf("\r\nget_key_event() para eroor!");
        while (1);
    }
    static PressEvent event_val = PRESS_DOWN;
    if (event_val != get_button_event(&key[key_num]))
    {
        event_val = get_button_event(&key[key_num]);
        switch (event_val)
        {
        case SINGLE_CLICK:      return KEY_SINGLE_CLICK;
        case DOUBLE_CLICK:      return KEY_DOUBLE_CLICK;
        case LONG_RRESS_START:  return KEY_LONG_RRESS_START;
        case LONG_PRESS_HOLD:   return KEY_LONG_PRESS_HOLD;
        default:                return KEY_NONE_PRESS;
        }
    }
    return KEY_NONE_PRESS;
}

inline void key_ticks(void)
{
    button_ticks();
}
