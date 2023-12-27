/**
 ******************************************************************************
 * @file    armcncio.h
 * @author  ARMCNC site:www.armcnc.net github:armcnc.github.io
 ******************************************************************************
 */
#ifndef __ARMCNCIO__
#define __ARMCNCIO__

#include <stdio.h>
#include <ctype.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "./include/wiringPi/piHiPri_c.h"
#include "./include/wiringPi/wiringPi.h"
#include "./include/wiringPi/wiringPi_c.h"
#include "./include/wiringPi/softPwm.h"
#include "./include/wiringPi/softPwm_c.h"

#include "rtapi.h"
#include "rtapi_app.h"
#include "rtapi_math.h"
#include "hal.h"

#define GPIO_BCM_MAX_COUNT 28
#define GPIO_PWM_MAX_COUNT 6
#define RTAPI_BIT(nr)(1UL << (nr))

#if defined(__arm__) || defined(__aarch64__)
#define ARMCNCIO_PLATFORM "arm64"
#endif

#if defined(__amd64__) || defined(__x86_64__)
#define ARMCNCIO_PLATFORM "amd64"
#endif

typedef struct
{
    hal_bit_t   *enable;

    hal_float_t *frequency_command;

    hal_float_t *duty_cycle_command;
    hal_float_t *duty_cycle_scale;

    hal_u32_t   *step_port;
    hal_bit_t   *step_pin;
    hal_bit_t   *step_pin_not;
    hal_u32_t   *step_direction_port;
    hal_bit_t   *step_direction_pin;
    hal_bit_t   *step_direction_pin_not;

    hal_u32_t   *spindle_pin;
    hal_bit_t   *spindle_pin_not;
    hal_u32_t   *spindle_forward_pin;
    hal_bit_t   *spindle_forward_pin_not;
    hal_u32_t   *spindle_reverse_pin;
    hal_bit_t   *spindle_reverse_pin_not;
}pwm_hal_struct;

typedef struct
{
    hal_bit_t   enable;

    hal_float_t frequency_command;

    hal_float_t duty_cycle_command;
    hal_float_t duty_cycle_scale;

    hal_u32_t   step_port;
    hal_bit_t   step_pin;
    hal_bit_t   step_pin_not;
    hal_u32_t   step_direction_port;
    hal_bit_t   step_direction_pin;
    hal_bit_t   step_direction_pin_not;

    hal_u32_t   spindle_pin;
    hal_bit_t   spindle_pin_not;
    hal_u32_t   spindle_forward_pin;
    hal_bit_t   spindle_forward_pin_not;
    hal_u32_t   spindle_reverse_pin;
    hal_bit_t   spindle_reverse_pin_not;

    int is_init;
} pwm_hal_priv_struct;

static uint32_t gpio_mask[GPIO_BCM_MAX_COUNT] = {0};
static uint32_t gpio_out_mask[GPIO_BCM_MAX_COUNT] = {0};
static uint32_t gpio_in_mask[GPIO_BCM_MAX_COUNT] = {0};

static hal_bit_t **gpio_hal;
static hal_bit_t gpio_hal_prev[GPIO_BCM_MAX_COUNT];
static int in_pins_array[GPIO_BCM_MAX_COUNT];
static int in_pins_count = 0;

static hal_bit_t **gpio_hal_not;
static hal_bit_t gpio_hal_not_prev[GPIO_BCM_MAX_COUNT];
static int out_pins_array[GPIO_BCM_MAX_COUNT];
static int out_pins_count = 0;

static pwm_hal_struct *pwm_hal;
static pwm_hal_priv_struct pwm_hal_prev[GPIO_PWM_MAX_COUNT];
static int pwm_hal_count = 0;

static void write_port(void *arg, long period);
static void read_port(void *arg, long period);
static void pwm_read(void *arg, long period);
static void pwm_write(void *arg, long period);

static int pwm_update_data(int ch)
{
    if (pwm_hal_prev[ch].enable != *pwm_hal[ch].enable) pwm_hal_prev[ch].enable = *pwm_hal[ch].enable;

    if (pwm_hal_prev[ch].frequency_command != *pwm_hal[ch].frequency_command) pwm_hal_prev[ch].frequency_command = *pwm_hal[ch].frequency_command;

    if (*pwm_hal[ch].duty_cycle_scale < 1e-20 && *pwm_hal[ch].duty_cycle_scale > -1e-20) *pwm_hal[ch].duty_cycle_scale = 1.0;

    if (pwm_hal_prev[ch].duty_cycle_scale != *pwm_hal[ch].duty_cycle_scale) pwm_hal_prev[ch].duty_cycle_scale = *pwm_hal[ch].duty_cycle_scale;

    if (pwm_hal_prev[ch].duty_cycle_command != *pwm_hal[ch].duty_cycle_command) pwm_hal_prev[ch].duty_cycle_command = *pwm_hal[ch].duty_cycle_command;

    if (pwm_hal_prev[ch].step_port != *pwm_hal[ch].step_port) pwm_hal_prev[ch].step_port = *pwm_hal[ch].step_port;

    if (pwm_hal_prev[ch].step_pin != *pwm_hal[ch].step_pin) pwm_hal_prev[ch].step_pin = *pwm_hal[ch].step_pin;

    if (pwm_hal_prev[ch].step_pin_not != *pwm_hal[ch].step_pin_not) pwm_hal_prev[ch].step_pin_not = *pwm_hal[ch].step_pin_not;

    if (pwm_hal_prev[ch].step_direction_port != *pwm_hal[ch].step_direction_port) pwm_hal_prev[ch].step_direction_port = *pwm_hal[ch].step_direction_port;

    if (pwm_hal_prev[ch].step_direction_pin != *pwm_hal[ch].step_direction_pin) pwm_hal_prev[ch].step_direction_pin = *pwm_hal[ch].step_direction_pin;

    if (pwm_hal_prev[ch].step_direction_pin_not != *pwm_hal[ch].step_direction_pin_not) pwm_hal_prev[ch].step_direction_pin_not = *pwm_hal[ch].step_direction_pin_not;

    if (pwm_hal_prev[ch].spindle_pin != *pwm_hal[ch].spindle_pin) pwm_hal_prev[ch].spindle_pin = *pwm_hal[ch].spindle_pin;

    if (pwm_hal_prev[ch].spindle_pin_not != *pwm_hal[ch].spindle_pin_not) pwm_hal_prev[ch].spindle_pin_not = *pwm_hal[ch].spindle_pin_not;

    if (pwm_hal_prev[ch].spindle_forward_pin != *pwm_hal[ch].spindle_forward_pin) pwm_hal_prev[ch].spindle_forward_pin = *pwm_hal[ch].spindle_forward_pin;

    if (pwm_hal_prev[ch].spindle_forward_pin_not != *pwm_hal[ch].spindle_forward_pin_not) pwm_hal_prev[ch].spindle_forward_pin_not = *pwm_hal[ch].spindle_forward_pin_not;

    if (pwm_hal_prev[ch].spindle_reverse_pin != *pwm_hal[ch].spindle_reverse_pin) pwm_hal_prev[ch].spindle_reverse_pin = *pwm_hal[ch].spindle_reverse_pin;

    if (pwm_hal_prev[ch].spindle_reverse_pin_not != *pwm_hal[ch].spindle_reverse_pin_not) pwm_hal_prev[ch].spindle_reverse_pin_not = *pwm_hal[ch].spindle_reverse_pin_not;

    return 0;
}

static int step_control(int ch)
{
    if (!pwm_hal_prev[ch].is_init)
    {
        pwm_update_data(ch);
        pwm_hal_prev[ch].is_init = 1;
        return 1;
    }

    digitalWrite((int)(*pwm_hal[ch].step_direction_port), *pwm_hal[ch].step_direction_pin ? HIGH : LOW);
    digitalWrite((int)(*pwm_hal[ch].step_port), *pwm_hal[ch].step_pin ? HIGH : LOW);
    
    return 0;
}

static int spindle_control(int ch)
{
    if (!pwm_hal_prev[ch].is_init)
    {
        softPwmCreate((int)(*pwm_hal[ch].spindle_pin), 0, 100);
        pwm_hal_prev[ch].is_init = 1;
        return 1;
    }

    pwm_update_data(ch);

    if (!(*pwm_hal[ch].enable))
    {
        //softPwmWrite((int)(*pwm_hal[ch].spindle_pin), 0);
        //digitalWrite((int)(*pwm_hal[ch].spindle_forward_pin), *pwm_hal[ch].spindle_forward_pin_not ? HIGH : LOW);
        //digitalWrite((int)(*pwm_hal[ch].spindle_reverse_pin), *pwm_hal[ch].spindle_reverse_pin_not ? HIGH : LOW);
    } else {
        int max_rpm = (int)(*pwm_hal[ch].duty_cycle_scale);
        int target_rpm = (int)(*pwm_hal[ch].duty_cycle_command);

        if (target_rpm < 0)
        {
            target_rpm = -target_rpm;
            digitalWrite((int)(*pwm_hal[ch].spindle_forward_pin), *pwm_hal[ch].spindle_forward_pin_not ? HIGH : LOW);
            digitalWrite((int)(*pwm_hal[ch].spindle_reverse_pin), *pwm_hal[ch].spindle_reverse_pin_not ? LOW : HIGH);
        } else {
            digitalWrite((int)(*pwm_hal[ch].spindle_forward_pin), *pwm_hal[ch].spindle_forward_pin_not ? LOW : HIGH);
            digitalWrite((int)(*pwm_hal[ch].spindle_reverse_pin), *pwm_hal[ch].spindle_reverse_pin_not ? HIGH : LOW);
        }

        int pwm_cycle = (target_rpm * 100) / max_rpm;

        softPwmWrite((int)(*pwm_hal[ch].spindle_pin), pwm_cycle);
    }

    return 0;
}

#endif