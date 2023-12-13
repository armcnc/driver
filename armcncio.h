/**
 ******************************************************************************
 * @file    armcncio.h
 * @author  ARMCNC site:www.armcnc.net github:armcnc.github.io
 ******************************************************************************
 */

#ifndef ARMCNCIO
#define ARMCNCIO

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

static hal_bit_t **gpio_in_out;
static hal_bit_t **gpio_in_out_not;
static hal_bit_t gpio_in_out_prev[GPIO_BCM_MAX_COUNT];
static hal_bit_t gpio_in_out_not_prev[GPIO_BCM_MAX_COUNT];
static int gpio_in_out_array[GPIO_BCM_MAX_COUNT];
static int gpio_in_array[] = {0};
static int gpio_out_array[] = {0};
static int gpio_in_out_count = 0;

static hal_s32_t **gpio_pull;
static hal_s32_t gpio_pull_prev[GPIO_BCM_MAX_COUNT];

static int32_t hal_malloc_init(void);
static void gpio_write(void *arg, long period);
static void gpio_read(void *arg, long period);
static void pwm_write(void *arg, long period);
static void pwm_read(void *arg, long period);

#endif