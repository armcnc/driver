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

#define GPIO_MAX_COUNT 40

#define RTAPI_BIT(nr) (1UL << (nr))

static hal_bit_t **gpio_hal_in;
static hal_bit_t **gpio_hal_in_not;
static hal_bit_t **gpio_hal_out;
static hal_bit_t **gpio_hal_out_not;
static hal_s32_t **gpio_hal_up_down;

static void gpio_write(void *arg, long period);
static void gpio_read(void *arg, long period);
static void pwm_write(void *arg, long period);
static void pwm_read(void *arg, long period);
