/**
 * @file bee_Led.c
 * @author nguyen__viet_hoang
 * @date 28 September 2023
 * @brief module for output with GPIO, API "create", "set level", "toggle" for others functions
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include <driver/gpio.h>
#include "driver/adc.h"
#include "driver/ledc.h"
#include "bee_Led.h"

static const adc1_channel_t adc_channel = ADC_CHANNEL_4;
static ledc_channel_config_t ledc_red_channel;
static ledc_channel_config_t ledc_green_channel;
static ledc_channel_config_t ledc_blue_channel;

void output_vCreate(gpio_num_t gpio_num)
{
    esp_rom_gpio_pad_select_gpio(gpio_num);
    // Set the GPIO as a push/pull output
    gpio_set_direction(gpio_num, GPIO_MODE_INPUT_OUTPUT);
}

void output_vSetLevel(gpio_num_t gpio_num, int level)
{
    gpio_set_level(gpio_num, level);
}

void output_vToggle(gpio_num_t gpio_num)
{
    int previous_level = gpio_get_level(gpio_num);
    gpio_set_level(gpio_num, 1 - previous_level);
}

void ledc_init_hardware()
{
    adc1_config_width(ADC_WIDTH_BIT_10);
    adc1_config_channel_atten(adc_channel, ADC_ATTEN_DB_11);
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 1000,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    ledc_timer_config(&ledc_timer);

    ledc_red_channel.channel = LEDC_CHANNEL_0;
    ledc_red_channel.duty = 0;
    ledc_red_channel.gpio_num = LEDC_RED;
    ledc_red_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_red_channel.hpoint = 0;
    ledc_red_channel.timer_sel = LEDC_TIMER_1;
    ledc_channel_config(&ledc_red_channel);

    ledc_green_channel.channel = LEDC_CHANNEL_1;
    ledc_green_channel.duty = 0;
    ledc_green_channel.gpio_num = LEDC_GREEN;
    ledc_green_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_green_channel.hpoint = 0;
    ledc_green_channel.timer_sel = LEDC_TIMER_1;
    ledc_channel_config(&ledc_green_channel);

    ledc_blue_channel.channel = LEDC_CHANNEL_2;
    ledc_blue_channel.duty = 0;
    ledc_blue_channel.gpio_num = LEDC_BLUE;
    ledc_blue_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_blue_channel.hpoint = 0;
    ledc_blue_channel.timer_sel = LEDC_TIMER_1;
    ledc_channel_config(&ledc_blue_channel);
}
void ledc_set_duty_rgb(uint8_t u8Red_value, uint8_t u8Green_value, uint8_t u8Blue_value)
{
    // Convert 8 bit to 10 bits
    uint16_t u16Red_value_10bit = (uint16_t)u8Red_value * 4;
    uint16_t u16Green_value_10bit = (uint16_t)u8Green_value * 4;
    uint16_t u16Blue_value_10bit = (uint16_t)u8Blue_value * 4;

    // Set value red ledc
    ledc_set_duty(ledc_red_channel.speed_mode, ledc_red_channel.channel, u16Red_value_10bit);
    ledc_update_duty(ledc_red_channel.speed_mode, ledc_red_channel.channel);

    // Set value green ledc
    ledc_set_duty(ledc_green_channel.speed_mode, ledc_green_channel.channel, u16Green_value_10bit);
    ledc_update_duty(ledc_green_channel.speed_mode, ledc_green_channel.channel);

    // Set value blue ledc
    ledc_set_duty(ledc_blue_channel.speed_mode, ledc_blue_channel.channel, u16Blue_value_10bit);
    ledc_update_duty(ledc_blue_channel.speed_mode, ledc_blue_channel.channel);
}

void ledc_fade_mode_task()
{
    uint8_t mark_led = 0;
    for (;;)
    {
        uint32_t adc_val = 0;
        for (int i = 0; i < SAMPLE_CNT; ++i)
        {
            adc_val += adc1_get_raw(adc_channel);
        }

        adc_val /= SAMPLE_CNT;

        if (adc_val == 0)
        {
            mark_led++;
            mark_led = mark_led % 3;
        }

        switch (mark_led)
        {
        case FLAG_LEDC_RED:
        {
            ledc_set_duty(ledc_red_channel.speed_mode, ledc_red_channel.channel, adc_val);
            ledc_update_duty(ledc_red_channel.speed_mode, ledc_red_channel.channel);
        }
        break;

        case FLAG_LEDC_GREEN:
        {
            ledc_set_duty(ledc_green_channel.speed_mode, ledc_green_channel.channel, adc_val);
            ledc_update_duty(ledc_green_channel.speed_mode, ledc_green_channel.channel);
        }
        break;

        case FLAG_LEDC_BLUE:
        {
            ledc_set_duty(ledc_blue_channel.speed_mode, ledc_blue_channel.channel, adc_val);
            ledc_update_duty(ledc_blue_channel.speed_mode, ledc_blue_channel.channel);
        }
        break;
        default:
            break;
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
void ledc_smooth_mode_task()
{
    uint8_t mark_led = 0;
    for (;;)
    {
        switch (mark_led)
        {
        case FLAG_LEDC_RED:
        {
            ledc_set_duty(ledc_red_channel.speed_mode, ledc_red_channel.channel, 1023);
            ledc_update_duty(ledc_red_channel.speed_mode, ledc_red_channel.channel);

            ledc_set_duty(ledc_green_channel.speed_mode, ledc_green_channel.channel, 0);
            ledc_update_duty(ledc_green_channel.speed_mode, ledc_green_channel.channel);

            ledc_set_duty(ledc_blue_channel.speed_mode, ledc_blue_channel.channel, 0);
            ledc_update_duty(ledc_blue_channel.speed_mode, ledc_blue_channel.channel);
        }
        break;
        case FLAG_LEDC_GREEN:
        {
            ledc_set_duty(ledc_red_channel.speed_mode, ledc_red_channel.channel, 0);
            ledc_update_duty(ledc_red_channel.speed_mode, ledc_red_channel.channel);

            ledc_set_duty(ledc_green_channel.speed_mode, ledc_green_channel.channel, 1023);
            ledc_update_duty(ledc_green_channel.speed_mode, ledc_green_channel.channel);

            ledc_set_duty(ledc_blue_channel.speed_mode, ledc_blue_channel.channel, 0);
            ledc_update_duty(ledc_blue_channel.speed_mode, ledc_blue_channel.channel);
        }
        break;
        case FLAG_LEDC_BLUE:
        {
            ledc_set_duty(ledc_red_channel.speed_mode, ledc_red_channel.channel, 0);
            ledc_update_duty(ledc_red_channel.speed_mode, ledc_red_channel.channel);

            ledc_set_duty(ledc_green_channel.speed_mode, ledc_green_channel.channel, 0);
            ledc_update_duty(ledc_green_channel.speed_mode, ledc_green_channel.channel);

            ledc_set_duty(ledc_blue_channel.speed_mode, ledc_blue_channel.channel, 1023);
            ledc_update_duty(ledc_blue_channel.speed_mode, ledc_blue_channel.channel);
        }
        default:
            break;
        }

        mark_led++;
        mark_led = mark_led % 3;
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}
void ledc_mix_fade_mode_task()
{
    uint8_t mark_led = 0;
    uint32_t adc_val_first_led = 0;
    uint32_t adc_val_second_led = 0;
    uint8_t u8Mark_inc_dec = FLAG_INCREASE;
    for (;;)
    {
        if (u8Mark_inc_dec == FLAG_INCREASE)
        {
            adc_val_first_led += STEP_INTENSITY_FIRST_LED;
            adc_val_second_led += STEP_INTENSITY_SECOND_LED;
        }
        else
        {
            adc_val_first_led -= STEP_INTENSITY_FIRST_LED;
            adc_val_second_led -= STEP_INTENSITY_SECOND_LED;
        }
        if (adc_val_first_led == HIGH_LEVEL_FIRST_LED || adc_val_second_led == HIGH_LEVEL_SECOND_LED)
        {
            u8Mark_inc_dec = FLAG_DECREASE;
        }
        else if (adc_val_first_led < LOW_LEVEL_FIRST_LED || adc_val_second_led < LOW_LEVEL_SECOND_LED)
        {
            u8Mark_inc_dec = FLAG_INCREASE;
            mark_led++;
            mark_led = mark_led % 3;
        }

        switch (mark_led)
        {
        case FLAG_LEDC_PURPLE:
        {
            ledc_set_duty(ledc_blue_channel.speed_mode, ledc_blue_channel.channel, adc_val_first_led);
            ledc_set_duty(ledc_red_channel.speed_mode, ledc_red_channel.channel, adc_val_second_led);
            ledc_update_duty(ledc_red_channel.speed_mode, ledc_red_channel.channel);
            ledc_update_duty(ledc_blue_channel.speed_mode, ledc_blue_channel.channel);
        }
        break;

        case FLAG_LEDC_YELLOW:
        {
            ledc_set_duty(ledc_green_channel.speed_mode, ledc_green_channel.channel, adc_val_first_led);
            ledc_set_duty(ledc_red_channel.speed_mode, ledc_red_channel.channel, adc_val_second_led);
            ledc_update_duty(ledc_red_channel.speed_mode, ledc_red_channel.channel);
            ledc_update_duty(ledc_green_channel.speed_mode, ledc_green_channel.channel);
        }
        break;

        case FLAG_LEDC_CYAN:
        {
            ledc_set_duty(ledc_blue_channel.speed_mode, ledc_blue_channel.channel, adc_val_first_led);
            ledc_set_duty(ledc_green_channel.speed_mode, ledc_green_channel.channel, adc_val_second_led);
            ledc_update_duty(ledc_blue_channel.speed_mode, ledc_blue_channel.channel);
            ledc_update_duty(ledc_green_channel.speed_mode, ledc_green_channel.channel);
        }
        break;
        default:
            break;
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/