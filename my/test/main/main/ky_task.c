#include "ky_task.h"
// #include "ky_lib.h"
// #include "lcd_st7789.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "key.h"



/************* LED任务 *************/
#define LED_PIN_NUM GPIO_NUM_23
void led_task(void)
{
    // gpio_pad_select_gpio(LED_PIN_NUM);
    // gpio_set_direction(LED_PIN_NUM, GPIO_MODE_OUTPUT);
    // gpio_set_level(LED_PIN_NUM, 0);

    while (1)
    {
    //     gpio_set_level(LED_PIN_NUM, 0);
    //     vTaskDelay(950 / portTICK_PERIOD_MS);

    //     gpio_set_level(LED_PIN_NUM, 1);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

/************* 按键测试任务 *************/
#define BUZZER_PIN_NUM GPIO_NUM_22
void key_test(void)
{
    gpio_pad_select_gpio(BUZZER_PIN_NUM);
    gpio_set_direction(BUZZER_PIN_NUM, GPIO_MODE_OUTPUT);
    gpio_set_level(BUZZER_PIN_NUM, 0);

    while (1)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        switch (get_key_event(0))
        {
        case KEY_SINGLE_CLICK:
            printf("单击\r\n");
            gpio_set_level(BUZZER_PIN_NUM, 1);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            gpio_set_level(BUZZER_PIN_NUM, 0);
            break;

        case KEY_DOUBLE_CLICK:
            printf("双击\r\n");
            gpio_set_level(BUZZER_PIN_NUM, 1);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            gpio_set_level(BUZZER_PIN_NUM, 0);

            vTaskDelay(500 / portTICK_PERIOD_MS);
            gpio_set_level(BUZZER_PIN_NUM, 1);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            gpio_set_level(BUZZER_PIN_NUM, 0);
            break;

        default:
            break;
        }
    }
}

/************* 按键任务 *************/
void key_task(void)
{
    key_init();

    while (1)
    {
        key_ticks();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

/************* LEDC任务 *************/
#define LEDC_PIN_NUM GPIO_NUM_23

#define LEDC_TEST_CH_NUM       (1)
#define LEDC_TEST_DUTY         (1024)
#define LEDC_TEST_FADE_TIME    (500)

void ledc_task(void)
{
    /* 配置定时器以确定PWM信号的频率和占空比分辨率 */
    ledc_timer_config_t ledc_timer;
    ledc_timer.duty_resolution = LEDC_TIMER_13_BIT; // 设置占空比分辨率
    ledc_timer.freq_hz = 5000;  // 设置频率
    ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_timer.timer_num = LEDC_TIMER_0;
    ledc_timer_config(&ledc_timer);
    
    /* 将定时器通道与GPIO相关联来配置通道，以输出PWM信号 */
    ledc_channel_config_t ledc_channel;
    ledc_channel.channel = LEDC_CHANNEL_0;          // 选择通道
    ledc_channel.duty = 0;                          // 初始化占空比。占空比=数值/占空比分辨率
    ledc_channel.gpio_num = LEDC_PIN_NUM;           // 关联GPIO引脚号
    ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE; // 选择速度模式
    ledc_channel.hpoint = 0;
    ledc_channel.timer_sel = LEDC_TIMER_0;          // 选择定时器
    ledc_channel_config(&ledc_channel);

    // Initialize fade service.
    ledc_fade_func_install(0);

    while (1)
    {
        // printf("1. LEDC fade up to duty = %d\n", LEDC_TEST_DUTY);

        ledc_set_fade_with_time(ledc_channel.speed_mode,
                ledc_channel.channel, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
        ledc_fade_start(ledc_channel.speed_mode,
                ledc_channel.channel, LEDC_FADE_NO_WAIT);
   
        vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

        // printf("2. LEDC fade down to duty = 0\n");

        ledc_set_fade_with_time(ledc_channel.speed_mode,
                ledc_channel.channel, 0, LEDC_TEST_FADE_TIME);
        ledc_fade_start(ledc_channel.speed_mode,
                ledc_channel.channel, LEDC_FADE_NO_WAIT);

        vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

        // printf("3. LEDC set duty = %d without fade\n", LEDC_TEST_DUTY);

        ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, LEDC_TEST_DUTY);
        ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);

        // vTaskDelay(10 / portTICK_PERIOD_MS);

        // printf("4. LEDC set duty = 0 without fade\n");

        ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 0);
        ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
 
        // vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}










void ky_create_task(void)
{
    /************* LED任务 *************/
    xTaskCreate(&led_task, "led_task", 512, NULL, 5, NULL);
    printf("LED任务创建成功\r\n");

    /************* 按键任务 *************/
    xTaskCreate(&key_task, "key_task", 2048, NULL, 1, NULL);
    printf("按键任务创建成功\r\n");

    /************* 按键测试任务 *************/
    xTaskCreate(&key_test, "key_test", 2048, NULL, 5, NULL);
    printf("按键测试任务创建成功\r\n");

    /************* LEDC测试任务 *************/
    xTaskCreate(&ledc_task, "ledc_task", 2048, NULL, 5, NULL);
    printf("LEDC任务创建成功\r\n");

    // /************* LCD测试任务 *************/
    // xTaskCreate(&lcd_task, "lcd_task", 10240, NULL, 5, NULL);
    // printf("LCD任务创建成功\r\n");

}

