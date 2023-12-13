/**
 ******************************************************************************
 * @file    armcncio.h
 * @author  ARMCNC site:www.armcnc.net github:armcnc.github.io
 ******************************************************************************
 */

#ifndef __ARMCNCIO__
#define __ARMCNCIO__

#include <stdio.h>
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
#define RTAPI_BIT(nr) (1UL << (nr))

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

static hal_bit_t **gpio_in_out;
static hal_bit_t **gpio_in_out_not;
static hal_bit_t gpio_in_out_prev[GPIO_BCM_MAX_COUNT];
static hal_bit_t gpio_in_out_not_prev[GPIO_BCM_MAX_COUNT];
static int gpio_in_out_array[GPIO_BCM_MAX_COUNT];
static int gpio_in_array[] = {0};
static int gpio_out_array[] = {0};
static int gpio_count = 0;

static hal_s32_t **gpio_pull;
static hal_s32_t gpio_pull_prev[GPIO_BCM_MAX_COUNT];

static pwm_hal_struct *pwm_hal;
static int pwm_array[GPIO_BCM_MAX_COUNT];
static int pwm_count = 0;

static void gpio_write(void *arg, long period);
static void gpio_read(void *arg, long period);
static void pwm_write(void *arg, long period);
static void pwm_read(void *arg, long period);

#endif