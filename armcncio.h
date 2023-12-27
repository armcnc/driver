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
#define GPIO_STEP_MAX_COUNT 5
#define GPIO_SPINDLE_MAX_COUNT 5
#define RTAPI_BIT(nr)(1UL << (nr))

#if defined(__arm__) || defined(__aarch64__)
#define ARMCNCIO_PLATFORM "arm64"
#endif

#if defined(__amd64__) || defined(__x86_64__)
#define ARMCNCIO_PLATFORM "amd64"
#endif

typedef struct
{
    hal_float_t *frequency_command;

    hal_float_t *duty_cycle_command;
    hal_float_t *duty_cycle_scale;

    hal_bit_t   *spindle_enable;
    hal_u32_t   *spindle_pin;
    hal_bit_t   *spindle_pin_not;
    hal_u32_t   *spindle_forward_pin;
    hal_bit_t   *spindle_forward_pin_not;
    hal_u32_t   *spindle_reverse_pin;
    hal_bit_t   *spindle_reverse_pin_not;
}spindle_hal_struct;

typedef struct
{
    hal_float_t frequency_command;

    hal_float_t duty_cycle_command;
    hal_float_t duty_cycle_scale;

    hal_bit_t   spindle_enable;
    hal_u32_t   spindle_pin;
    hal_bit_t   spindle_pin_not;
    hal_u32_t   spindle_forward_pin;
    hal_bit_t   spindle_forward_pin_not;
    hal_u32_t   spindle_reverse_pin;
    hal_bit_t   spindle_reverse_pin_not;

    int is_init;
} spindle_hal_priv_struct;

typedef struct
{
    hal_bit_t   *step_enable;
    hal_u32_t   *step_port;
    hal_bit_t   *step_pin;
    hal_bit_t   *step_pin_not;
    hal_u32_t   *step_direction_port;
    hal_bit_t   *step_direction_pin;
    hal_bit_t   *step_direction_pin_not;
}step_hal_struct;

typedef struct
{
    hal_bit_t   step_enable;
    hal_u32_t   step_port;
    hal_bit_t   step_pin;
    hal_bit_t   step_pin_not;
    hal_u32_t   step_direction_port;
    hal_bit_t   step_direction_pin;
    hal_bit_t   step_direction_pin_not;
    
    int is_init;
} step_hal_priv_struct;

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

static spindle_hal_struct *spindle_hal;
static spindle_hal_priv_struct spindle_hal_prev[GPIO_SPINDLE_MAX_COUNT];
static int spindle_hal_count = 0;

static step_hal_struct *step_hal;
static step_hal_priv_struct step_hal_prev[GPIO_STEP_MAX_COUNT];
static int step_hal_count = 0;

static void write_port(void *arg, long period);
static void read_port(void *arg, long period);
static void step_port(void *arg, long period);
static void spindle_port(void *arg, long period);

static int spindle_update_data(int ch)
{
    if (spindle_hal_prev[ch].spindle_enable != *spindle_hal[ch].spindle_enable) spindle_hal_prev[ch].spindle_enable = *spindle_hal[ch].spindle_enable;

    if (spindle_hal_prev[ch].frequency_command != *spindle_hal[ch].frequency_command) spindle_hal_prev[ch].frequency_command = *spindle_hal[ch].frequency_command;

    if (*spindle_hal[ch].duty_cycle_scale < 1e-20 && *spindle_hal[ch].duty_cycle_scale > -1e-20) *spindle_hal[ch].duty_cycle_scale = 1.0;

    if (spindle_hal_prev[ch].duty_cycle_scale != *spindle_hal[ch].duty_cycle_scale) spindle_hal_prev[ch].duty_cycle_scale = *spindle_hal[ch].duty_cycle_scale;

    if (spindle_hal_prev[ch].duty_cycle_command != *spindle_hal[ch].duty_cycle_command) spindle_hal_prev[ch].duty_cycle_command = *spindle_hal[ch].duty_cycle_command;

    if (spindle_hal_prev[ch].spindle_pin != *spindle_hal[ch].spindle_pin) spindle_hal_prev[ch].spindle_pin = *spindle_hal[ch].spindle_pin;

    if (spindle_hal_prev[ch].spindle_pin_not != *spindle_hal[ch].spindle_pin_not) spindle_hal_prev[ch].spindle_pin_not = *spindle_hal[ch].spindle_pin_not;

    if (spindle_hal_prev[ch].spindle_forward_pin != *spindle_hal[ch].spindle_forward_pin) spindle_hal_prev[ch].spindle_forward_pin = *spindle_hal[ch].spindle_forward_pin;

    if (spindle_hal_prev[ch].spindle_forward_pin_not != *spindle_hal[ch].spindle_forward_pin_not) spindle_hal_prev[ch].spindle_forward_pin_not = *spindle_hal[ch].spindle_forward_pin_not;

    if (spindle_hal_prev[ch].spindle_reverse_pin != *spindle_hal[ch].spindle_reverse_pin) spindle_hal_prev[ch].spindle_reverse_pin = *spindle_hal[ch].spindle_reverse_pin;

    if (spindle_hal_prev[ch].spindle_reverse_pin_not != *spindle_hal[ch].spindle_reverse_pin_not) spindle_hal_prev[ch].spindle_reverse_pin_not = *spindle_hal[ch].spindle_reverse_pin_not;

    return 0;
}

static int step_update_data(int ch)
{
    if (step_hal_prev[ch].step_enable != *step_hal[ch].step_enable) step_hal_prev[ch].step_enable = *step_hal[ch].step_enable;

    if (step_hal_prev[ch].step_port != *step_hal[ch].step_port) step_hal_prev[ch].step_port = *step_hal[ch].step_port;

    if (step_hal_prev[ch].step_pin != *step_hal[ch].step_pin) step_hal_prev[ch].step_pin = *step_hal[ch].step_pin;

    if (step_hal_prev[ch].step_pin_not != *step_hal[ch].step_pin_not) step_hal_prev[ch].step_pin_not = *step_hal[ch].step_pin_not;

    if (step_hal_prev[ch].step_direction_port != *step_hal[ch].step_direction_port) step_hal_prev[ch].step_direction_port = *step_hal[ch].step_direction_port;

    if (step_hal_prev[ch].step_direction_pin != *step_hal[ch].step_direction_pin) step_hal_prev[ch].step_direction_pin = *step_hal[ch].step_direction_pin;

    if (step_hal_prev[ch].step_direction_pin_not != *step_hal[ch].step_direction_pin_not) step_hal_prev[ch].step_direction_pin_not = *step_hal[ch].step_direction_pin_not;

    return 0;
}

static int step_control(int ch)
{
    if (!step_hal_prev[ch].is_init)
    {
        step_update_data(ch);
        step_hal_prev[ch].is_init = 1;
        return 1;
    }

    digitalWrite((int)(*step_hal[ch].step_direction_port), *step_hal[ch].step_direction_pin ? HIGH : LOW);
    digitalWrite((int)(*step_hal[ch].step_port), *step_hal[ch].step_pin ? HIGH : LOW);
    
    return 0;
}

static int spindle_control(int ch)
{
    if (!spindle_hal_prev[ch].is_init)
    {
        spindle_update_data(ch);
        softPwmCreate((int)(*spindle_hal[ch].spindle_pin), 0, 100);
        softPwmWrite((int)(*spindle_hal[ch].spindle_pin), 0);
        digitalWrite((int)(*spindle_hal[ch].spindle_forward_pin), *spindle_hal[ch].spindle_forward_pin_not ? HIGH : LOW);
        digitalWrite((int)(*spindle_hal[ch].spindle_reverse_pin), *spindle_hal[ch].spindle_reverse_pin_not ? HIGH : LOW);
        spindle_hal_prev[ch].is_init = 1;
        return 1;
    }

    if (!(*spindle_hal[ch].spindle_enable))
    {
        softPwmWrite((int)(*spindle_hal[ch].spindle_pin), 0);
        digitalWrite((int)(*spindle_hal[ch].spindle_forward_pin), *spindle_hal[ch].spindle_forward_pin_not ? HIGH : LOW);
        digitalWrite((int)(*spindle_hal[ch].spindle_reverse_pin), *spindle_hal[ch].spindle_reverse_pin_not ? HIGH : LOW);
    } else {
        int max_rpm = (int)(*spindle_hal[ch].duty_cycle_scale);
        int target_rpm = (int)(*spindle_hal[ch].duty_cycle_command);

        if (target_rpm < 0)
        {
            target_rpm = -target_rpm;
            digitalWrite((int)(*spindle_hal[ch].spindle_forward_pin), *spindle_hal[ch].spindle_forward_pin_not ? HIGH : LOW);
            digitalWrite((int)(*spindle_hal[ch].spindle_reverse_pin), *spindle_hal[ch].spindle_reverse_pin_not ? LOW : HIGH);
        } else {
            digitalWrite((int)(*spindle_hal[ch].spindle_forward_pin), *spindle_hal[ch].spindle_forward_pin_not ? LOW : HIGH);
            digitalWrite((int)(*spindle_hal[ch].spindle_reverse_pin), *spindle_hal[ch].spindle_reverse_pin_not ? HIGH : LOW);
        }

        int pwm_cycle = (target_rpm * 100) / max_rpm;

        softPwmWrite((int)(*spindle_hal[ch].spindle_pin), pwm_cycle);
    }

    return 0;
}

#endif