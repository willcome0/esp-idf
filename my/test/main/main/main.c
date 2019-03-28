
#include <stdio.h>

// #include "freertos/FreeRTOS.h"

// #include "freertos/task.h"

// #include "driver/gpio.h"

// #include "sdkconfig.h"

// #include "multi_button.h"
#include "ky_task.h"
#include "lcd.h"


void app_main()
{
    // ky_create_task();
    printf("初始化成功\r\n");

    lcd_test();
}
