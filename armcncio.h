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
    hal_bit_t *enable;

    hal_float_t *freq_cmd;

    hal_u32_t *pwm_pin;

    hal_u32_t *forward_pin;
    hal_bit_t *forward_pin_not;

    hal_u32_t *reverse_pin;
    hal_bit_t *reverse_pin_not;

    hal_float_t *dc_cmd;
    hal_float_t *dc_scale;

}pwm_hal_struct;

typedef struct
{
    hal_bit_t enable;

    hal_float_t freq_cmd;

    hal_u32_t pwm_pin;

    hal_u32_t forward_pin;
    hal_bit_t forward_pin_not;

    hal_u32_t reverse_pin;
    hal_bit_t reverse_pin_not;

    hal_float_t dc_cmd;
    hal_float_t dc_scale;

    hal_u32_t ctrl_type;
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
static pwm_hal_priv_struct pwm_hal_prev[GPIO_BCM_MAX_COUNT];
static int pwm_hal_array[GPIO_BCM_MAX_COUNT];
static int pwm_hal_count = 0;

static void gpio_write(void *arg, long period);
static void gpio_read(void *arg, long period);
static void pwm_write(void *arg, long period);
static void pwm_read(void *arg, long period);

#endif