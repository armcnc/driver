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
    hal_bit_t *enable; // in

    hal_u32_t *pwm_port; // in
    hal_u32_t *pwm_pin; // in
    hal_bit_t *pwm_invert; // in

    hal_u32_t *dir_port; // in
    hal_u32_t *dir_pin; // in
    hal_bit_t *dir_invert; // in
    hal_u32_t *dir_hold; // io
    hal_u32_t *dir_setup; // io

    hal_float_t *dc_cmd; // in
    hal_float_t *dc_scale; // io
    hal_float_t *dc_min; // io
    hal_float_t *dc_max; // io
    hal_u32_t   *dc_max_t; // io
    hal_float_t *dc_offset; // io

    hal_float_t *pos_cmd; // in
    hal_float_t *pos_scale; // io

    hal_float_t *vel_cmd; // in
    hal_float_t *vel_scale; // io

    hal_float_t *freq_cmd; // io
    hal_float_t *freq_min; // io
    hal_float_t *freq_max; // io

    hal_float_t *dc_fb; // out
    hal_float_t *pos_fb; // out
    hal_float_t *vel_fb; // out
    hal_float_t *freq_fb; // out
    hal_s32_t   *counts; // out
}pwm_hal_struct;

typedef struct
{
    hal_bit_t enable; // in

    hal_u32_t pwm_port; // in
    hal_u32_t pwm_pin; // in
    hal_bit_t pwm_invert; // in

    hal_u32_t dir_port; // in
    hal_u32_t dir_pin; // in
    hal_bit_t dir_invert; // in
    hal_u32_t dir_hold; // io
    hal_u32_t dir_setup; // io

    hal_float_t dc_cmd; // in
    hal_float_t dc_scale; // io
    hal_float_t dc_min; // io
    hal_float_t dc_max; // io
    hal_u32_t dc_max_t; // io
    hal_float_t dc_offset; // io

    hal_float_t pos_cmd; // in
    hal_float_t pos_scale; // io

    hal_float_t vel_cmd; // in
    hal_float_t vel_scale; // io

    hal_float_t freq_cmd; // io
    hal_float_t freq_min; // io
    hal_float_t freq_max; // io

    hal_float_t dc_fb; // out
    hal_float_t pos_fb; // out
    hal_float_t vel_fb; // out
    hal_float_t freq_fb; // out
    hal_s32_t counts; // out

    hal_u32_t ctrl_type;
    hal_s32_t freq_mHz;
    hal_u32_t freq_min_mHz;
    hal_u32_t freq_max_mHz;
    hal_s32_t dc_s32;
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
static pwm_hal_priv_struct pwm_hal_prev[GPIO_BCM_MAX_COUNT];
static int pwm_hal_array[GPIO_BCM_MAX_COUNT];
static int pwm_hal_count = 0;

static void gpio_write(void *arg, long period);
static void gpio_read(void *arg, long period);
static void pwm_write(void *arg, long period);
static void pwm_read(void *arg, long period);

static void pwm_pins_update(int ch)
{
    uint32_t upd = 0;

    if (pwm_hal_prev[ch].pwm_port != *pwm_hal[ch].pwm_port) {pwm_hal_prev[ch].pwm_port = *pwm_hal[ch].pwm_port; upd++;}
    if (pwm_hal_prev[ch].pwm_pin != *pwm_hal[ch].pwm_pin) {pwm_hal_prev[ch].pwm_pin = *pwm_hal[ch].pwm_pin; upd++;}
    if (pwm_hal_prev[ch].pwm_invert != *pwm_hal[ch].pwm_invert) {pwm_hal_prev[ch].pwm_invert = *pwm_hal[ch].pwm_invert; upd++;}

    if (pwm_hal_prev[ch].dir_port != *pwm_hal[ch].dir_port) {pwm_hal_prev[ch].dir_port = *pwm_hal[ch].dir_port; upd++;}
    if (pwm_hal_prev[ch].dir_pin != *pwm_hal[ch].dir_pin) {pwm_hal_prev[ch].dir_pin = *pwm_hal[ch].dir_pin; upd++;}

    if (pwm_hal_prev[ch].dir_invert != *pwm_hal[ch].dir_invert) {pwm_hal_prev[ch].dir_invert = *pwm_hal[ch].dir_invert; upd++;}

    if (upd)
    {
        softPwmCreate((int)(*pwm_hal[ch].pwm_pin), 0, (int)(10000 / (*pwm_hal[ch].freq_cmd)));
    }
}

static int32_t pwm_get_new_dc(int ch)
{
    if (*pwm_hal[ch].dc_cmd == pwm_hal_prev[ch].dc_cmd &&
        *pwm_hal[ch].dc_scale == pwm_hal_prev[ch].dc_scale && 
        *pwm_hal[ch].dc_offset == pwm_hal_prev[ch].dc_offset && 
        *pwm_hal[ch].dc_min == pwm_hal_prev[ch].dc_min && 
        *pwm_hal[ch].dc_max == pwm_hal_prev[ch].dc_max ) return pwm_hal_prev[ch].dc_s32;
    
    if (*pwm_hal[ch].dc_min < -1.0) *pwm_hal[ch].dc_min = -1.0;
    if (*pwm_hal[ch].dc_max > 1.0) *pwm_hal[ch].dc_max = 1.0;
    if (*pwm_hal[ch].dc_max < *pwm_hal[ch].dc_min) *pwm_hal[ch].dc_max = *pwm_hal[ch].dc_min;
    if (*pwm_hal[ch].dc_scale < 1e-20 && *pwm_hal[ch].dc_scale > -1e-20) *pwm_hal[ch].dc_scale = 1.0;

    *pwm_hal[ch].dc_fb = *pwm_hal[ch].dc_cmd / *pwm_hal[ch].dc_scale + *pwm_hal[ch].dc_offset;

    if (*pwm_hal[ch].dc_fb < *pwm_hal[ch].dc_min) *pwm_hal[ch].dc_fb = *pwm_hal[ch].dc_min;
    if (*pwm_hal[ch].dc_fb > *pwm_hal[ch].dc_max) *pwm_hal[ch].dc_fb = *pwm_hal[ch].dc_max;

    pwm_hal_prev[ch].dc_cmd = *pwm_hal[ch].dc_cmd;
    pwm_hal_prev[ch].dc_min = *pwm_hal[ch].dc_min;
    pwm_hal_prev[ch].dc_max = *pwm_hal[ch].dc_max;
    pwm_hal_prev[ch].dc_offset = *pwm_hal[ch].dc_offset;
    pwm_hal_prev[ch].dc_scale = *pwm_hal[ch].dc_scale;

    return (int32_t) (*pwm_hal[ch].dc_fb * INT32_MAX);
}

static int32_t pwm_get_new_freq(int ch, long period)
{
    int32_t freq = 0;

    if (pwm_hal_prev[ch].freq_min != *pwm_hal[ch].freq_min)
    {
        pwm_hal_prev[ch].freq_min_mHz = (hal_u32_t) round(*pwm_hal[ch].freq_min * 1000);
        pwm_hal_prev[ch].freq_min = *pwm_hal[ch].freq_min;
    }

    if (pwm_hal_prev[ch].freq_max != *pwm_hal[ch].freq_max)
    {
        pwm_hal_prev[ch].freq_max_mHz = (hal_u32_t) round(*pwm_hal[ch].freq_max * 1000);
        pwm_hal_prev[ch].freq_max = *pwm_hal[ch].freq_max;
    }

    switch (pwm_hal_prev[ch].ctrl_type)
    {
        case 1: {
            break;
        }
        case 2: {
            if (pwm_hal_prev[ch].freq_cmd == *pwm_hal[ch].freq_cmd)
            {
                freq = pwm_hal_prev[ch].freq_mHz;
                break;
            }
            pwm_hal_prev[ch].freq_cmd = *pwm_hal[ch].freq_cmd;
            if (*pwm_hal[ch].freq_cmd < 1e-20 && *pwm_hal[ch].freq_cmd > -1e-20) break;
            freq = (int32_t)round(*pwm_hal[ch].freq_cmd * 1000);
            if (abs(freq) < pwm_hal_prev[ch].freq_min_mHz) freq = 0;
            else if ( abs(freq) > pwm_hal_prev[ch].freq_max_mHz ) freq = pwm_hal_prev[ch].freq_max_mHz * (freq < 0 ? -1 : 1);
            break;
        }
    }

    *pwm_hal[ch].freq_fb = freq ? ((hal_float_t) freq) / 1000 : 0.0;

    return freq;
}

#endif