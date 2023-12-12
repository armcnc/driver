/**
 ******************************************************************************
 * @file    armcncio.h
 * @author  ARMCNC site:www.armcnc.net github:armcnc.github.io
 ******************************************************************************
 */

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

#define GPIO_PORTS_MAX_CNT 1
#define GPIO_PINS_MAX_CNT 27
#define PWM_CH_MAX_CNT 27

static const char *gpio_name[GPIO_PORTS_MAX_CNT] = {"PI"};

static hal_bit_t **gpio_hal[GPIO_PORTS_MAX_CNT];
static hal_bit_t gpio_hal_prev[GPIO_PORTS_MAX_CNT][GPIO_PINS_MAX_CNT];

static hal_bit_t **gpio_hal_not[GPIO_PORTS_MAX_CNT];
static hal_bit_t gpio_hal_not_prev[GPIO_PORTS_MAX_CNT][GPIO_PINS_MAX_CNT];

static hal_s32_t **gpio_hal_pull[GPIO_PORTS_MAX_CNT];
static hal_s32_t gpio_hal_pull_prev[GPIO_PORTS_MAX_CNT][GPIO_PINS_MAX_CNT];

static hal_u32_t **gpio_hal_drive[GPIO_PORTS_MAX_CNT];
static hal_u32_t gpio_hal_drive_prev[GPIO_PORTS_MAX_CNT][GPIO_PINS_MAX_CNT];

static uint32_t gpio_out_mask[GPIO_PORTS_MAX_CNT] = {0};
static uint32_t gpio_in_mask[GPIO_PORTS_MAX_CNT] = {0};

static uint32_t gpio_in_cnt = 0;
static uint32_t gpio_out_cnt = 0;
static uint32_t gpio_ports_cnt = 0;
static uint32_t gpio_pins_cnt[GPIO_PINS_MAX_CNT] = {0};
static uint32_t pin_msk[GPIO_PINS_MAX_CNT] = {0};

typedef struct
{
    hal_bit_t *enable; // in

    hal_u32_t *pwm_port; // in
    hal_u32_t *pwm_pin; // in
    hal_bit_t *pwm_inv; // in

    hal_u32_t *dir_port; // in
    hal_u32_t *dir_pin; // in
    hal_bit_t *dir_inv; // in
    hal_u32_t *dir_hold; // io
    hal_u32_t *dir_setup; // io

    hal_float_t *dc_cmd; // in
    hal_float_t *dc_scale; // io
    hal_float_t *dc_min; // io
    hal_float_t *dc_max; // io
    hal_u32_t *dc_max_t; // io
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
    hal_s32_t *counts; // out
} pwm_ch_hal_t;

typedef struct
{
    hal_bit_t enable; // in

    hal_u32_t pwm_port; // in
    hal_u32_t pwm_pin; // in
    hal_bit_t pwm_inv; // in

    hal_u32_t dir_port; // in
    hal_u32_t dir_pin; // in
    hal_bit_t dir_inv; // in
    hal_u32_t dir_hold; // io
    hal_u32_t dir_setup; // io

    hal_float_t dc_cmd; // in
    hal_float_t dc_scale; // io
    hal_float_t dc_min; // io
    hal_float_t dc_max; // io
    hal_u32_t *dc_max_t; // io
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
} pwm_ch_private_t;

enum
{
    PWM_CTRL_BY_POS,
    PWM_CTRL_BY_VEL,
    PWM_CTRL_BY_FREQ
};

static pwm_ch_hal_t *pwmh;
static pwm_ch_private_t pwmp[PWM_CH_MAX_CNT] = {0};
static uint8_t pwm_ch_cnt = 0;

#define pwm_hal *pwmh[ch]
#define pwm_private pwmp[ch]

static void gpio_write(void *arg, long period);
static void gpio_read(void *arg, long period);
static void pwm_write(void *arg, long period);
static void pwm_read(void *arg, long period);
