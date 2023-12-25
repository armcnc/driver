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

typedef struct
{
    hal_bit_t   *enable;

    hal_float_t *frequency_command;
    hal_float_t *frequency_min;
    hal_float_t *frequency_max;
    hal_float_t *frequency_feedback;

    hal_float_t *duty_cycle_command;
    hal_float_t *duty_cycle_scale;
    hal_u32_t   *duty_cycle_max_time;
    hal_float_t *duty_cycle_min;
    hal_float_t *duty_cycle_max;
    hal_float_t *duty_cycle_offset;
    hal_float_t *duty_cycle_feedback;

    hal_float_t *position_command;
    hal_float_t *position_scale;
    hal_float_t *position_feedback;
    hal_s32_t   *position_count;
    hal_float_t *position_current;

    hal_u32_t   *pwm_pin;
    hal_bit_t   *pwm_pin_not;

    hal_u32_t   *step_direction_pin;
    hal_bit_t   *step_direction_pin_not;
    hal_u32_t   *step_direction_hold_time;
    hal_u32_t   *step_direction_setup_time;

    hal_u32_t   *spindle_forward_pin;
    hal_bit_t   *spindle_forward_pin_not;

    hal_u32_t   *spindle_reverse_pin;
    hal_bit_t   *spindle_reverse_pin_not;
}pwm_hal_struct;

typedef struct
{
    hal_bit_t   enable;

    hal_float_t frequency_command;
    hal_float_t frequency_min;
    hal_float_t frequency_max;
    hal_float_t frequency_feedback;

    hal_float_t duty_cycle_command;
    hal_float_t duty_cycle_scale;
    hal_u32_t   duty_cycle_max_time;
    hal_float_t duty_cycle_min;
    hal_float_t duty_cycle_max;
    hal_float_t duty_cycle_offset;
    hal_float_t duty_cycle_feedback;

    hal_float_t position_command;
    hal_float_t position_scale;
    hal_float_t position_feedback;
    hal_s32_t   position_count;
    hal_float_t position_current;

    hal_u32_t   pwm_pin;
    hal_bit_t   pwm_pin_not;

    hal_u32_t   step_direction_pin;
    hal_bit_t   step_direction_pin_not;
    hal_u32_t   step_direction_hold_time;
    hal_u32_t   step_direction_setup_time;

    hal_u32_t   spindle_forward_pin;
    hal_bit_t   spindle_forward_pin_not;

    hal_u32_t   spindle_reverse_pin;
    hal_bit_t   spindle_reverse_pin_not;

    int ctrl_type;
    int is_init;
    hal_s32_t duty_cycle_s32;
    hal_s32_t freq_mHz;
    hal_u32_t freq_min_mHz;
    hal_u32_t freq_max_mHz;
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

static hal_s32_t **gpio_hal_pull;
static hal_s32_t gpio_hal_pull_prev[GPIO_BCM_MAX_COUNT];

static hal_u32_t **gpio_hal_drive;
static hal_u32_t gpio_hal_drive_prev[GPIO_BCM_MAX_COUNT];

static pwm_hal_struct *pwm_hal;
static pwm_hal_priv_struct pwm_hal_prev[GPIO_BCM_MAX_COUNT];
static int pwm_hal_array[GPIO_BCM_MAX_COUNT];
static int pwm_hal_count = 0;

static void gpio_write(void *arg, long period);
static void gpio_read(void *arg, long period);
static void pwm_write(void *arg, long period);
static void pwm_read(void *arg, long period);

static int pwm_update_data(int ch)
{
    if (pwm_hal_prev[ch].enable != *pwm_hal[ch].enable)
    {
        pwm_hal_prev[ch].enable = *pwm_hal[ch].enable;
    }

    if (pwm_hal_prev[ch].frequency_command != *pwm_hal[ch].frequency_command)
    {
        pwm_hal_prev[ch].frequency_command = *pwm_hal[ch].frequency_command;
    }

    if (*pwm_hal[ch].duty_cycle_scale < 1e-20 && *pwm_hal[ch].duty_cycle_scale > -1e-20)
    {
        *pwm_hal[ch].duty_cycle_scale = 1.0;
    }

    if (pwm_hal_prev[ch].duty_cycle_scale != *pwm_hal[ch].duty_cycle_scale)
    {
        pwm_hal_prev[ch].duty_cycle_scale = *pwm_hal[ch].duty_cycle_scale;
    }

    if (pwm_hal_prev[ch].pwm_pin != *pwm_hal[ch].pwm_pin)
    {
        pwm_hal_prev[ch].pwm_pin = *pwm_hal[ch].pwm_pin;
    }

    if (pwm_hal_prev[ch].pwm_pin_not != *pwm_hal[ch].pwm_pin_not)
    {
        pwm_hal_prev[ch].pwm_pin_not = *pwm_hal[ch].pwm_pin_not;
    }

    if (pwm_hal_prev[ch].step_direction_pin != *pwm_hal[ch].step_direction_pin)
    {
        pwm_hal_prev[ch].step_direction_pin = *pwm_hal[ch].step_direction_pin;
    }

    if (pwm_hal_prev[ch].step_direction_pin_not != *pwm_hal[ch].step_direction_pin_not)
    {
        pwm_hal_prev[ch].step_direction_pin_not = *pwm_hal[ch].step_direction_pin_not;
    }

    if (pwm_hal_prev[ch].spindle_forward_pin != *pwm_hal[ch].spindle_forward_pin)
    {
        pwm_hal_prev[ch].spindle_forward_pin = *pwm_hal[ch].spindle_forward_pin;
    }

    if (pwm_hal_prev[ch].spindle_forward_pin_not != *pwm_hal[ch].spindle_forward_pin_not)
    {
        pwm_hal_prev[ch].spindle_forward_pin_not = *pwm_hal[ch].spindle_forward_pin_not;
    }

    if (pwm_hal_prev[ch].spindle_reverse_pin != *pwm_hal[ch].spindle_reverse_pin)
    {
        pwm_hal_prev[ch].spindle_reverse_pin = *pwm_hal[ch].spindle_reverse_pin;
    }

    if (pwm_hal_prev[ch].spindle_reverse_pin_not != *pwm_hal[ch].spindle_reverse_pin_not)
    {
        pwm_hal_prev[ch].spindle_reverse_pin_not = *pwm_hal[ch].spindle_reverse_pin_not;
    }

    return 0;
}

static int pwm_step_control(int ch, long period)
{
    if (!pwm_hal_prev[ch].is_init)
    {
        softPwmCreate((int)(*pwm_hal[ch].pwm_pin), 0, 100);
        softPwmWrite((int)(*pwm_hal[ch].pwm_pin), 0);
        digitalWrite((int)(*pwm_hal[ch].step_direction_pin), *pwm_hal[ch].step_direction_pin_not ? HIGH : LOW);
        pwm_hal_prev[ch].is_init = 1;
        return 1;
    }

    pwm_update_data(ch);

    if (!(*pwm_hal[ch].enable))
    {
        softPwmWrite((int)(*pwm_hal[ch].pwm_pin), 0);
        digitalWrite((int)(*pwm_hal[ch].step_direction_pin), *pwm_hal[ch].step_direction_pin_not ? HIGH : LOW);
    } else {
        long targetSteps = (*pwm_hal[ch].position_command) * (*pwm_hal[ch].position_scale);
        long currentSteps = (*pwm_hal[ch].position_current) * (*pwm_hal[ch].position_scale);

        int direction = (targetSteps > currentSteps) ? HIGH : LOW;
        digitalWrite((int)(*pwm_hal[ch].step_direction_pin), direction);

        if (currentSteps != targetSteps) {
            softPwmWrite((int)(*pwm_hal[ch].pwm_pin), 50); // 设定占空比
            delayMicroseconds(*pwm_hal[ch].duty_cycle_max_time);
            softPwmWrite((int)(*pwm_hal[ch].pwm_pin), 0);

            if (direction == HIGH) {
                (*pwm_hal[ch].position_current) += 1.0 / (*pwm_hal[ch].position_scale);
            } else {
                (*pwm_hal[ch].position_current) -= 1.0 / (*pwm_hal[ch].position_scale);
            }

            (*pwm_hal[ch].position_feedback) = (*pwm_hal[ch].position_current);
        }
    }

    return 0;
}

static int pwm_spindle_control(int ch)
{
    if (!pwm_hal_prev[ch].is_init)
    {
        softPwmCreate((int)(*pwm_hal[ch].pwm_pin), 0, 100);
        softPwmWrite((int)(*pwm_hal[ch].pwm_pin), 0);
        digitalWrite((int)(*pwm_hal[ch].spindle_forward_pin), *pwm_hal[ch].spindle_forward_pin_not ? HIGH : LOW);
        digitalWrite((int)(*pwm_hal[ch].spindle_reverse_pin), *pwm_hal[ch].spindle_reverse_pin_not ? HIGH : LOW);
        pwm_hal_prev[ch].is_init = 1;
        return 1;
    }

    pwm_update_data(ch);

    if (!(*pwm_hal[ch].enable))
    {
        softPwmWrite((int)(*pwm_hal[ch].pwm_pin), 0);
        pullUpDnControl((int)(*pwm_hal[ch].spindle_forward_pin), PUD_OFF);
        digitalWrite((int)(*pwm_hal[ch].spindle_forward_pin), *pwm_hal[ch].spindle_forward_pin_not ? HIGH : LOW);
        pullUpDnControl((int)(*pwm_hal[ch].spindle_reverse_pin), PUD_OFF);
        digitalWrite((int)(*pwm_hal[ch].spindle_reverse_pin), *pwm_hal[ch].spindle_reverse_pin_not ? HIGH : LOW);
    } else {
        int max_rpm = (int)(*pwm_hal[ch].duty_cycle_scale);
        int target_rpm = (int)(*pwm_hal[ch].duty_cycle_command);

        if (target_rpm < 0)
        {
            target_rpm = -target_rpm;
            pullUpDnControl((int)(*pwm_hal[ch].spindle_forward_pin), PUD_OFF);
            digitalWrite((int)(*pwm_hal[ch].spindle_forward_pin), *pwm_hal[ch].spindle_forward_pin_not ? HIGH : LOW);
            pullUpDnControl((int)(*pwm_hal[ch].spindle_reverse_pin), PUD_OFF);
            digitalWrite((int)(*pwm_hal[ch].spindle_reverse_pin), *pwm_hal[ch].spindle_reverse_pin_not ? LOW : HIGH);
        } else {
            pullUpDnControl((int)(*pwm_hal[ch].spindle_forward_pin), PUD_OFF);
            digitalWrite((int)(*pwm_hal[ch].spindle_forward_pin), *pwm_hal[ch].spindle_forward_pin_not ? LOW : HIGH);
            pullUpDnControl((int)(*pwm_hal[ch].spindle_reverse_pin), PUD_OFF);
            digitalWrite((int)(*pwm_hal[ch].spindle_reverse_pin), *pwm_hal[ch].spindle_reverse_pin_not ? HIGH : LOW);
        }

        int pwm_cycle = (target_rpm * 100) / max_rpm;

        softPwmWrite((int)(*pwm_hal[ch].pwm_pin), pwm_cycle);
    }

    return 0;
}

#endif