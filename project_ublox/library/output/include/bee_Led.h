/**
 * @file bee_Led.h
 * @author nguyen__viet_hoang
 * @date 25 June 2023
 * @brief module for output with GPIO, API "create", "set level", "toggle" for others functions
 */
#ifndef BEE_LED_H
#define BEE_LED_H

#include "esp_err.h"
#include "hal/gpio_types.h"

#define LED_CONNECTED_BROKER GPIO_NUM_2
#define LED_HIGH_LEVEL 1
#define LED_LOW_LEVEL 0

#define SAMPLE_CNT 16

#define LEDC_BLUE GPIO_NUM_27
#define LEDC_GREEN GPIO_NUM_26
#define LEDC_RED GPIO_NUM_25

#define FLAG_LEDC_RED 0
#define FLAG_LEDC_GREEN 1
#define FLAG_LEDC_BLUE 2

#define FLAG_LEDC_PURPLE 1
#define FLAG_LEDC_YELLOW 0
#define FLAG_LEDC_CYAN 2

#define FLAG_INCREASE 0
#define FLAG_DECREASE 1

#define LOW_LEVEL_FIRST_LED 5
#define LOW_LEVEL_SECOND_LED 25
#define HIGH_LEVEL_FIRST_LED 100
#define HIGH_LEVEL_SECOND_LED 1020

#define STEP_INTENSITY_FIRST_LED 5
#define STEP_INTENSITY_SECOND_LED 25

void output_vCreate(gpio_num_t gpio_num);
void output_vSetLevel(gpio_num_t gpio_num, int level);
void output_vToggle(gpio_num_t gpio_num);

void ledc_init_hardware();

void ledc_fade_mode_task();
void ledc_mix_fade_mode_task();
void ledc_smooth_mode_task();

void ledc_set_duty_rgb(uint8_t u8Red_value, uint8_t u8Green_value, uint8_t u8Blue_value);

#endif