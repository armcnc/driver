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
static pwm_hal_priv_struct pwm_hal_priv[GPIO_BCM_MAX_COUNT] = {0};
static int pwm_types_array[GPIO_BCM_MAX_COUNT];
static int pwm_types_count = 0;

#define pwm_hal_var *pwm_hal[ch]
#define pwm_private_var pwm_hal_priv[ch]

static void gpio_write(void *arg, long period);
static void gpio_read(void *arg, long period);
static void pwm_write(void *arg, long period);
static void pwm_read(void *arg, long period);

static void pwm_pins_update(int ch)
{
    uint32_t upd = 0;

    if (pwm_private_var.pwm_port != pwm_hal_var.pwm_port) {pwm_private_var.pwm_port = pwm_hal_var.pwm_port; upd++;}
    if (pwm_private_var.pwm_pin != pwm_hal_var.pwm_pin) {pwm_private_var.pwm_pin = pwm_hal_var.pwm_pin; upd++;}
    if (pwm_private_var.pwm_invert != pwm_hal_var.pwm_invert) {pwm_private_var.pwm_invert = pwm_hal_var.pwm_invert; upd++;}

    if (pwm_private_var.dir_port != pwm_hal_var.dir_port) {pwm_private_var.dir_port = pwm_hal_var.dir_port; upd++;}
    if (pwm_private_var.dir_pin != pwm_hal_var.dir_pin) {pwm_private_var.dir_pin = pwm_hal_var.dir_pin; upd++;}

    if (pwm_private_var.dir_invert != pwm_hal_var.dir_invert) {pwm_private_var.dir_invert = pwm_hal_var.dir_invert; upd++;}

    if (upd)
    {
        softPwmCreate((int)pwm_hal_var.pwm_pin, 0, 500);
    }
}

static int32_t pwm_get_new_dc(uint8_t ch)
{
    if (pwm_hal_var.dc_cmd == pwm_private_var.dc_cmd &&
        pwm_hal_var.dc_scale == pwm_private_var.dc_scale && 
        pwm_hal_var.dc_offset == pwm_private_var.dc_offset && 
        pwm_hal_var.dc_min == pwm_private_var.dc_min && 
        pwm_hal_var.dc_max == pwm_private_var.dc_max ) return pwm_private_var.dc_s32;
    
    if (pwm_hal_var.dc_min < -1.0) pwm_hal_var.dc_min = -1.0;
    if (pwm_hal_var.dc_max > 1.0) pwm_hal_var.dc_max = 1.0;
    if (pwm_hal_var.dc_max < pwm_hal_var.dc_min) pwm_hal_var.dc_max = pwm_hal_var.dc_min;
    if (pwm_hal_var.dc_scale < 1e-20 && pwm_hal_var.dc_scale > -1e-20) pwm_hal_var.dc_scale = 1.0;

    pwm_hal_var.dc_fb = pwm_hal_var.dc_cmd / pwm_hal_var.dc_scale + pwm_hal_var.dc_offset;

    if (pwm_hal_var.dc_fb < pwm_hal_var.dc_min) pwm_hal_var.dc_fb = pwm_hal_var.dc_min;
    if (pwm_hal_var.dc_fb > pwm_hal_var.dc_max) pwm_hal_var.dc_fb = pwm_hal_var.dc_max;

    pwm_private_var.dc_cmd = pwm_hal_var.dc_cmd;
    pwm_private_var.dc_min = pwm_hal_var.dc_min;
    pwm_private_var.dc_max = pwm_hal_var.dc_max;
    pwm_private_var.dc_offset = pwm_hal_var.dc_offset;
    pwm_private_var.dc_scale = pwm_hal_var.dc_scale;

    return (int32_t) (pwm_hal_var.dc_fb * INT32_MAX);
}

#endif