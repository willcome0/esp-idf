#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

// #include "pretty_effect.h"
#include "font.h"
#include "lcd.h"
// #define LCD_TOTAL_BUF_SIZE	(240*240*2)
#define LCD_BUF_SIZE (1152)
uint8_t *lcd_buf;

#define LCD_Width 240
#define LCD_Height 240

#define PIN_NUM_MISO 25
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK 19
#define PIN_NUM_CS 22

#define PIN_NUM_DC 21
#define PIN_NUM_RST 18
#define PIN_NUM_BCKL 5

//To speed up transfers, every SPI transfer sends a bunch of lines. This define specifies how many. More means more memory use,
//but less overhead for setting up / finishing transfers. Make sure 240 is dividable by this.
#define PARALLEL_LINES 16

spi_device_handle_t spi;
#define LCD_SPI spi
/*
 The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct.
*/
typedef struct
{
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

typedef enum
{
    LCD_TYPE_ILI = 1,
    LCD_TYPE_ST,
    LCD_TYPE_MAX,
} type_lcd_t;

//Place data into DRAM. Constant data gets placed into DROM by default, which is not accessible by DMA.
DRAM_ATTR static const lcd_init_cmd_t st_init_cmds[] = {
    /* Memory Data Access Control, MX=MV=0, MY=ML=MH=0, RGB=0 */
    {0x36, {0x00}, 1},
    /* 0x13正常；0x21反显 */
    {0x21, {0x13}, 1},
    /* Interface Pixel Format, 16bits/pixel for RGB/MCU interface */
    {0x3A, {0x65}, 1},
    /* Porch Setting */
    {0xB2, {0x0c, 0x0c, 0x00, 0x33, 0x33}, 5},
    /* Gate Control, Vgh=13.65V, Vgl=-10.43V */
    {0xB7, {0x72}, 1},
    /* VCOM Setting, VCOM=1.175V */
    {0xBB, {0x3D}, 1},
    /* LCM Control, XOR: BGR, MX, MH */
    {0xC0, {0x2C}, 1},
    /* VDV and VRH Command Enable, enable=1 */
    {0xC2, {0x01}, 1},
    /* VRH Set, Vap=4.4+... */
    {0xC3, {0x19}, 1},
    /* VDV Set, VDV=0 */
    {0xC4, {0x20}, 1},
    /* Frame Rate Control, 60Hz, inversion=0 */
    {0xC6, {0x0f}, 1},
    /* Power Control 1, AVDD=6.8V, AVCL=-4.8V, VDDS=2.3V */
    {0xD0, {0xA4, 0xA1}, 2},
    /* Positive Voltage Gamma Control */
    {0xE0, {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23}, 14},
    /* Negative Voltage Gamma Control */
    {0xE1, {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23}, 14},
    /* Sleep Out */
    {0x11, {0}, 0x80},
    /* Display On */
    {0x29, {0}, 0x80},

    /* Memory Data Access Control, MX=MV=1, MY=ML=MH=0, RGB=0 */
};

void lcd_cmd(const uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));                       //Zero out the transaction
    t.length = 8;                                   //Command is 8 bits
    t.tx_buffer = &cmd;                             //The data is the cmd itself
    t.user = (void *)0;                             //D/C needs to be set to 0
    ret = spi_device_polling_transmit(LCD_SPI, &t); //Transmit!
    assert(ret == ESP_OK);                          //Should have had no issues.
}

void lcd_data(const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len == 0)
        return;                                     //no need to send anything
    memset(&t, 0, sizeof(t));                       //Zero out the transaction
    t.length = len * 8;                             //Len is in bytes, transaction length is in bits.
    t.tx_buffer = data;                             //Data
    t.user = (void *)1;                             //D/C needs to be set to 1
    ret = spi_device_polling_transmit(LCD_SPI, &t); //Transmit!
    assert(ret == ESP_OK);                          //Should have had no issues.
}

void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc = (int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}

void lcd_init(void)
{
    int cmd = 0;
    const lcd_init_cmd_t *lcd_init_cmds;

    //Initialize non-SPI GPIOs
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);

    //Reset the display
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);

    lcd_init_cmds = st_init_cmds;

    //Send all the commands
    while (lcd_init_cmds[cmd].databytes != 0xff)
    {
        lcd_cmd(lcd_init_cmds[cmd].cmd);                                        // 鍙戦€佸懡浠?
        lcd_data(lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes & 0x1F); // 鍙戦€佹暟鎹?
        if (lcd_init_cmds[cmd].databytes & 0x80)
        {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }

    ///Enable backlight
    gpio_set_level(PIN_NUM_BCKL, 0);
}

void lcd_address_set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    esp_err_t ret;
    uint16_t temp;
    if (x1>x2) 
    {
        temp = x1;
        x1 = x2;
        x2 = temp;
    }
    if (y1>y2)
    {
        temp = y1;
        y1 = y2;
        y2 = temp;
    }
    static spi_transaction_t trans[6];

    for (uint8_t x = 0; x < 5; x++)
    {
        memset(&trans[x], 0, sizeof(spi_transaction_t));
        if ((x & 1) == 0)
        {
            //Even transfers are commands
            trans[x].length = 8;
            trans[x].user = (void *)0;
        }
        else
        {
            //Odd transfers are data
            trans[x].length = 8 * 4;
            trans[x].user = (void *)1;
        }
        trans[x].flags = SPI_TRANS_USE_TXDATA;
    }
    trans[0].tx_data[0] = 0x2A;    //Column Address Set
    trans[1].tx_data[0] = x1 >> 8; //Start Col High
    trans[1].tx_data[1] = x1;      //Start Col Low
    trans[1].tx_data[2] = x2 >> 8; //End Col High
    trans[1].tx_data[3] = x2;      //End Col Low

    trans[2].tx_data[0] = 0x2B;    //Page address set
    trans[3].tx_data[0] = y1 >> 8; //Start page high
    trans[3].tx_data[1] = y1;      //start page low
    trans[3].tx_data[2] = y2 >> 8; //end page high
    trans[3].tx_data[3] = y2;      //end page low

    trans[4].tx_data[0] = 0x2C; //memory write

    ret = spi_device_queue_trans(LCD_SPI, &trans[0], portMAX_DELAY);
    assert(ret == ESP_OK);
    ret = spi_device_queue_trans(LCD_SPI, &trans[1], portMAX_DELAY);
    assert(ret == ESP_OK);
    ret = spi_device_queue_trans(LCD_SPI, &trans[2], portMAX_DELAY);
    assert(ret == ESP_OK);
    ret = spi_device_queue_trans(LCD_SPI, &trans[3], portMAX_DELAY);
    assert(ret == ESP_OK);
    ret = spi_device_queue_trans(LCD_SPI, &trans[4], portMAX_DELAY);
    assert(ret == ESP_OK);
}

spi_transaction_t trans;

void lcd_fill(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint16_t color)
{
    uint32_t size = 0, size_remain = 0;

    size = (x_end - x_start + 1) * (y_end - y_start + 1) * 2;

    if (size > LCD_BUF_SIZE)
    {
        size_remain = size - LCD_BUF_SIZE;
        size = LCD_BUF_SIZE;
    }

    lcd_address_set(x_start, y_start, x_end, y_end);

    while (1)
    {
        for (uint16_t i = 0; i < size / 2; i++)
        {
            lcd_buf[2 * i] = color >> 8;
            lcd_buf[2 * i + 1] = color;
        }

        trans.tx_buffer = lcd_buf;
        trans.length = size * 8; // 鍗曚綅涓烘瘮鐗?
        trans.rxlength = 0;      // 涓嶅姞浼氬脊鍑洪敊璇?細 rxdata transfer > host maximum
        trans.rx_buffer = NULL;  // 涓嶅姞涓€浼氫細鍗℃?
        trans.flags = 0;
        trans.user = (void *)1;

        esp_err_t ret = spi_device_queue_trans(LCD_SPI, &trans, portMAX_DELAY);
        assert(ret == ESP_OK);

        if (size_remain == 0)
            break;

        if (size_remain > LCD_BUF_SIZE)
        {
            size_remain = size_remain - LCD_BUF_SIZE;
        }
        else
        {
            size = size_remain;
            size_remain = 0;
        }
    }
}
void lcd_clear(uint16_t color)
{

    lcd_address_set(0, 0, 239, 239);

    for (int i = 0; i < LCD_BUF_SIZE / 2; i++)
    {
        lcd_buf[i * 2] = color >> 8;
        lcd_buf[i * 2 + 1] = color;
    }

    trans.tx_buffer = lcd_buf;
    trans.length = LCD_BUF_SIZE * 8; // 鍗曚綅涓烘瘮鐗?
    trans.rxlength = 0;              // 涓嶅姞浼氬脊鍑洪敊璇?細 rxdata transfer > host maximum
    trans.rx_buffer = NULL;          // 涓嶅姞涓€浼氫細鍗℃?
    trans.flags = 0;
    trans.user = (void *)1;

    for (int i = 0; i <= 100; i++) // bug:100会填充不满？
    {
        esp_err_t ret = spi_device_queue_trans(LCD_SPI, &trans, portMAX_DELAY);
        assert(ret == ESP_OK);
    }
    // lcd_fill(0, 0, LCD_Width-1, LCD_Height-1, color);
}
void lcd_write_color(uint16_t color)
{
    trans.length = 8 * 2;
    trans.user = (void *)1;
    trans.flags = SPI_TRANS_USE_TXDATA;
    trans.tx_data[0] = color >> 8;
    trans.tx_data[1] = color;
    trans.rxlength = 0;
    trans.rx_buffer = NULL;
    spi_device_queue_trans(LCD_SPI, &trans, portMAX_DELAY);
}

void lcd_draw_point(uint16_t x, uint16_t y, uint16_t color)
{
    lcd_address_set(x, y, x, y);
    lcd_write_color(color);
}
void lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, row, col;
    


    if (y1 == y2)
    {
        lcd_address_set(x1, y1, x2, y2);

        for (uint16_t i = 0; i < abs(x2 - x1); i++)
        {
            lcd_buf[2 * i] = color >> 8;
            lcd_buf[2 * i + 1] = color;
        }

        trans.tx_buffer = lcd_buf;
        trans.length = abs(x2 - x1) * 2 * 8; // 鍗曚綅涓烘瘮鐗?
        trans.rxlength = 0;               // rxdata transfer > host maximum
        trans.rx_buffer = NULL;           //
        trans.flags = 0;
        trans.user = (void *)1;

        esp_err_t ret = spi_device_queue_trans(LCD_SPI, &trans, portMAX_DELAY);
        assert(ret == ESP_OK);
        return;
    }

    delta_x = x2 - x1;
    delta_y = y2 - y1;
    row = x1;
    col = y1;

    if (delta_x > 0)
        incx = 1;

    else if (delta_x == 0)
        incx = 0;

    else
    {
        incx = -1;
        delta_x = -delta_x;
    }

    if (delta_y > 0)
        incy = 1;

    else if (delta_y == 0)
        incy = 0;

    else
    {
        incy = -1;
        delta_y = -delta_y;
    }

    if (delta_x > delta_y)
        distance = delta_x;

    else
        distance = delta_y;

    for (uint16_t t = 0; t <= distance + 1; t++)
    {
        lcd_draw_point(row, col, color);
        xerr += delta_x;
        yerr += delta_y;

        if (xerr > distance)
        {
            xerr -= distance;
            row += incx;
        }

        if (yerr > distance)
        {
            yerr -= distance;
            col += incy;
        }
    }
}

void lcd_draw_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    lcd_draw_line(x1, y1, x2, y1, color);
    lcd_draw_line(x1, y1, x1, y2, color);
    lcd_draw_line(x1, y2, x2, y2, color);
    lcd_draw_line(x2, y1, x2, y2, color);
}

void lcd_show_en(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, char num, uint16_t size, uint8_t cover_flag)
{
    uint8_t data;

    lcd_address_set(x, y, x + size / 2 - 1, y + size - 1);

    uint8_t row_num = size;
    uint8_t col_num = size / 2 / 8 + ((size / 2) % 8 != 0 ? 1 : 0);

    for (uint8_t j = 0; j < row_num; j++)
    {
        for (uint8_t i = 0; i < col_num; i++)
        {
            switch (size)
            {

            case 16:
                data = EN_FONT16[(num - ' ') * 16 + j * col_num + i];
                break;
            default:
                return;
            }

            for (uint8_t k = 0; k < ((i == col_num - 1) ? ((size / 2) % 8 != 0 ? (size / 2) % 8 : 8) : 8); k++)
            {
                if (cover_flag)
                {
                    if (data & 0x01)
                        lcd_write_color(fc);
                    else
                        lcd_write_color(bc);
                    data >>= 1;
                }
                else
                {
                    if (data & 0x01)
                        lcd_draw_point(x + i * 8 + k, y + j, fc);
                    data >>= 1;
                }
            }
        }
    }
}

void lcd_show_zh(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, char *str, uint16_t size, uint8_t cover_flag)
{
    uint16_t zh_num;
    uint8_t data;
    uint8_t index[3];
    uint8_t RowNum = size;
    uint8_t ColNum = size / 8 + (size % 8 != 0 ? 1 : 0);

    switch (size)
    {
    case 16:
        zh_num = ZH_FONT16_NUM;
        break;

    default:
        return;
    }
    for (uint16_t k = 0; k < zh_num; k++)
    {
        switch (size)
        {
        case 16:
            index[0] = ZH_FONT16[k].index[0];
            index[1] = ZH_FONT16[k].index[1];
            index[2] = ZH_FONT16[k].index[2];
            break;

        default:
            return;
        }
        if ((index[0] == *(str)) && (index[1] == *(str + 1) && (index[2] == *(str + 2))))
        {
            lcd_address_set(x, y, x + size - 1, y + size - 1);

            for (uint8_t j = 0; j < RowNum; j++)
            {
                for (uint8_t i = 0; i < ColNum; i++)
                {
                    switch (size)
                    {
                    case 16:
                        data = ZH_FONT16[k].msk[j * ColNum + i];
                        break;
                    default:
                        return;
                    }

                    for (uint8_t a = 0; a < ((i == ColNum - 1) ? ((size) % 8 != 0 ? (size) % 8 : 8) : 8); a++)
                    {
                        if (cover_flag)
                        {
                            if (data & 0x01)
                                lcd_write_color(fc);
                            else
                                lcd_write_color(bc);
                            data >>= 1;
                        }
                        else
                        {
                            if (data & 0x01)
                                lcd_draw_point(x + i * 8 + a, y + j, fc);
                            data >>= 1;
                        }
                    }
                }
            }
        }
        continue;
    }
}

void lcd_show_str(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, char *str, uint16_t size, uint8_t cover_flag)
{
    uint8_t bHz = 0;

    while (*str != 0)
    {
        if (!bHz)
        {
            if (*str > 0x80)
                bHz = 1;
            else
            {

                lcd_show_en(x, y, fc, bc, *str, size, cover_flag);
                x += (size / 2);

                str++;
            }
        }
        else
        {
            bHz = 0;

            lcd_show_zh(x, y, fc, bc, str, size, cover_flag);
            str += 3;
            x += size;
        }
    }
}

void lcd_show_pic(uint16_t x, uint16_t y, const uint8_t *p, uint16_t x_size, uint16_t y_size)
{
    uint8_t picH, picL;

    lcd_address_set(x, y, x + x_size - 1, y + y_size - 1);
    static spi_transaction_t trans;

    // uint8_t *data = p;
    // for (uint32_t i = 0; i < x_size * y_size * 2 / LCD_BUF_SIZE; i++)
    // {
    //     trans.tx_buffer = gImage_start + i * LCD_BUF_SIZE;
    //     trans.length = LCD_BUF_SIZE * 8;
    //     trans.rxlength = 0;     // rxdata transfer > host maximum
    //     trans.rx_buffer = NULL; //
    //     trans.flags = 0;
    //     trans.user = (void *)1;

    //     esp_err_t ret = spi_device_queue_trans(LCD_SPI, &trans, portMAX_DELAY);
    //     assert(ret == ESP_OK);
    // }
    // trans.tx_buffer = p+x_size*y_size*2-((x_size*y_size*2)%LCD_BUF_SIZE);
    // trans.length = (x_size*y_size*2)%LCD_BUF_SIZE;
    // trans.rxlength = 0;               // rxdata transfer > host maximum
    // trans.rx_buffer = NULL;           //
    // trans.flags = 0;
    // trans.user = (void *)1;

    // esp_err_t ret = spi_device_queue_trans(LCD_SPI, &trans, portMAX_DELAY);

    for (uint16_t i = 0; i < x_size * y_size; i++)
    {

        picL = *(p + i * 2);
        picH = *(p + i * 2 + 1);
        lcd_write_color(picH << 8 | picL);
    }
}
void lcd_draw_circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
    int a = 0;
    int b = r;
    int di = 3 - (r << 1);

    while (a <= b)
    {
        // printf("a:%d, b:%d\r\n", a, b);
        // if (a==83)
        //     color = RED;
        lcd_draw_point(x0 - b, y0 - a, color);
        lcd_draw_point(x0 + b, y0 - a, color);
        lcd_draw_point(x0 - a, y0 + b, color);

        lcd_draw_point(x0 - b, y0 - a, color);
        lcd_draw_point(x0 - a, y0 - b, color);
        lcd_draw_point(x0 + b, y0 + a, color);

        lcd_draw_point(x0 + a, y0 - b, color);
        lcd_draw_point(x0 + a, y0 + b, color);
        lcd_draw_point(x0 - b, y0 + a, color);

        printf("%3d %3d | %3d %3d | %3d %3d | %3d %3d | %3d %3d | %3d %3d | %3d %3d | %3d %3d | %3d %3d", x0 - b, y0 - a, x0 + b, y0 - a, x0 - a, y0 + b, x0 - b, y0 - a, x0 - a, y0 - b, x0 + b, y0 + a, x0 + a, y0 - b, x0 + a, y0 + b, x0 - b, y0 + a);

        a++;

        if (di < 0)
            di += 4 * a + 6;
        else
        {
            di += 10 + 4 * (a - b);
            b--;
        }

        lcd_draw_point(x0 + a, y0 + b, color);
        printf(" * %3d %3d\r\n", x0 + a, y0 + b);
    }
}

void lcd_test()
{
    esp_err_t ret;

    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_BUF_SIZE};

    spi_device_interface_config_t devcfg = {

        .clock_speed_hz = 25 * 1000 * 1000,      //Clock out at 10 MHz
        .mode = 0,                               //SPI mode 0
        .spics_io_num = PIN_NUM_CS,              //CS pin
        .queue_size = 7,                         //We want to be able to queue 7 transactions at a time
        .pre_cb = lcd_spi_pre_transfer_callback, //Specify pre-transfer callback to handle D/C line
        .post_cb = NULL,
    };
    ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    lcd_init();

    int frame = 0;

    uint16_t color[5] = {WHITE, BLACK, RED, GREEN, BLUE};
    lcd_buf = heap_caps_malloc(LCD_BUF_SIZE, MALLOC_CAP_DMA);
    assert(lcd_buf != NULL);

    while (1)
    {
        // memset(lines, color[frame%10], LCD_BUF_SIZE);
        frame++;

        // lcd_clear(spi, BLUE);

        lcd_clear(BLACK);

//     {
//         // vTaskDelay(10 / portTICK_RATE_MS);
//         lcd_draw_circle(119, 119, 119, WHITE);
//         lcd_draw_circle(119, 119, 95, WHITE);

//         #define _C 24
//         #define _A 1
//         #define _B 1
//         lcd_draw_line(119,   0,    119, 0 + _C, RED); // 上
//         lcd_draw_line(118,   0,    118, 0 + _C, RED);
//         lcd_draw_line(120,   0,    120, 0 + _C, RED);

//         lcd_draw_line(238, 119, 238-_C,    119, RED); // 右
//         lcd_draw_line(238, 118, 238-_C,    118, RED);
//         lcd_draw_line(238, 120, 238-_C,    120, RED);

//         lcd_draw_line(119, 238,    119, 238-_C, RED); // 下
//         lcd_draw_line(118, 238,    118, 238-_C, RED);
//         lcd_draw_line(120, 238,    120, 238-_C, RED);

//         lcd_draw_line(  0, 119, 0 + _C,    119, RED); // 左
//         lcd_draw_line(  0, 118, 0 + _C,    118, RED);
//         lcd_draw_line(  0, 120, 0 + _C,    120, RED);

//         // lcd_draw_line(178,  16, 178-_A,  16+_B, RED);

//         // lcd_draw_line(222,  60, 222-_A,  60+_B, RED);

//         // lcd_draw_line(222, 178, 222-_A, 178-_B, RED);

//         // lcd_draw_line(178, 222, 178-_A, 222-_B, RED);
// ///////////////////// 中线相连
//         lcd_draw_line(178,  16, 60,  222, BLUE);
        
//         lcd_draw_line(222,  60, 16,  178, BLUE);

//         lcd_draw_line(222, 178, 16, 60, BLUE);

//         lcd_draw_line(178, 222, 60, 16, BLUE);


//         lcd_draw_line(166,  37, 72,  201, BLUE);
    
//         lcd_draw_line(201,  72, 37,  166, BLUE);

//         lcd_draw_line(201,  166, 37,  72, BLUE);

//         lcd_draw_line(166,  201, 72,  37, BLUE);


//     lcd_draw_line(177,  16, 165,  36, GREEN);
//     lcd_draw_line(221,  59, 200,  71, GREEN);
//     lcd_draw_line(222,  177, 202,  165, GREEN);
//     lcd_draw_line(179,  221, 167,  200, GREEN);

//     lcd_draw_line(59,  221, 71, 200, GREEN);
//     lcd_draw_line(16,  177, 36, 165, GREEN);
//     lcd_draw_line(17,  59, 38, 71, GREEN);
//     lcd_draw_line(61,  16, 73, 36, GREEN);

// //////////////////////// 上线相连
//     // lcd_draw_line(177,  16, 59,  221, GREEN);
//     // lcd_draw_line(221,  59, 16,  177, GREEN);
//     // lcd_draw_line(222,  177, 17,  59, GREEN);
//     // lcd_draw_line(179,  221, 61,  16, GREEN);

// // 连好的线
//         // lcd_draw_line(178,  16, 166,  37, RED);
        
//         // lcd_draw_line(222,  60, 201,  72, RED);

//         // lcd_draw_line(222, 178, 201,  166, RED);

//         // lcd_draw_line(178, 222, 166,  201, RED);

//         // lcd_draw_line(60,  222, 72,  201, RED);
        
//         // lcd_draw_line(16,  178, 37,  166, RED);

//         // lcd_draw_line(16, 60, 37,  72, RED);

//         // lcd_draw_line(60, 16, 72,  37, RED);

//             // lcd_draw_line(15, 59, 37, 72, RED);
//         // lcd_draw_line(14, 58, 36, 72, RED);
//         // lcd_draw_line(13, 57, 35, 72, RED);
//         // lcd_draw_line(16, 60, 38, 72, RED);

//         // lcd_draw_line(59, 15, 72, 37, RED);
//         // lcd_draw_line(58, 14, 71, 37, RED);
//         // lcd_draw_line(57, 13, 70, 37, RED);
//         // lcd_draw_line(60, 16, 73, 37, RED);

//         // lcd_draw_line( 36,  35,   36+18,  35+18, RED);    // 左上
//         // lcd_draw_line( 35,  34,   35+17,  34+17, RED);
//         // lcd_draw_line( 37,  36,   37+17,  36+17, RED);

//         // lcd_draw_line(202,  35,   202-18,  35+18, RED);    // 右上
//         // lcd_draw_line(201,  34,   201-17,  34+17, RED);
//         // lcd_draw_line(203,  36,   203-17,  36+17, RED);

//         // lcd_draw_line( 36, 203,   36+18,  203-18, RED);    // 左下
//         // lcd_draw_line( 35, 202,   35+17,  202-17, RED);
//         // lcd_draw_line( 37, 204,   37+17,  204-17, RED);

//         // lcd_draw_line(202, 203,   202-18,  203-18, RED);    // 右下
//         // lcd_draw_line(201, 202,   201-17,  202-17, RED);
//         // lcd_draw_line(203, 204,   203-17,  204-17, RED);

//         // lcd_draw_line(119, 0, 119, 0+30, WHITE);
//         // lcd_draw_line(119, 0, 119, 0+30, WHITE);
//         vTaskDelay(20000 / portTICK_RATE_MS);

//     }
        // lcd_clear(color[frame % 5]);
        // // vTaskDelay(100 / portTICK_RATE_MS);
        // lcd_fill(0, 0, 239, 239, color[(frame + 2) % 5]);
        // // vTaskDelay(100 / portTICK_RATE_MS);

        // lcd_draw_line(0, 119, 240, 119, WHITE);
        // lcd_draw_line(119, 0, 119, 240, WHITE);
        // lcd_draw_line(0, 0, 240, 240, WHITE);
        // lcd_draw_line(240, 0, 0, 240, WHITE);
        // // vTaskDelay(100 / portTICK_RATE_MS);

        // lcd_draw_rect(16, 16, 32, 32, WHITE);
        // // vTaskDelay(100 / portTICK_RATE_MS);
        // lcd_show_str(0, 0, WHITE, BLACK, "Hello World! 你好世界", 16, 1);

        lcd_show_pic(0, 0, gImage_iwatch, 240, 240);
        vTaskDelay(20000 / portTICK_RATE_MS);

        printf("一帧图片\r\n");
    }
}
