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

#include "./lib/wiringPi/piHiPri_c.h"
#include "./lib/wiringPi/wiringPi.h"
#include "./lib/wiringPi/wiringPi_c.h"
#include "./lib/wiringPi/softPwm.h"
#include "./lib/wiringPi/softPwm_c.h"

#include "rtapi.h"
#include "rtapi_app.h"
#include "rtapi_math.h"
#include "hal.h"

#define GPIO_PORTS_MAX_CNT 8
#define GPIO_PINS_MAX_CNT 24
#define PWM_CH_MAX_CNT 16

#define ph *pwmh[ch] // `ph` means `PWM HAL`
#define pp pwmp[ch] // `pp` means `PWM Private`

#define EXPORT_PIN(IO_TYPE,VAR_TYPE,VAL,NAME,DEFAULT) \
    r += hal_pin_##VAR_TYPE##_newf(IO_TYPE, &(pwmh[ch].VAL), comp_id,\
    "%s.pwm.%d." NAME, comp_name, ch);\
    ph.VAL = DEFAULT;\
    pp.VAL = DEFAULT;

enum
{
    PWM_CTRL_BY_POS,
    PWM_CTRL_BY_VEL,
    PWM_CTRL_BY_FREQ
};

typedef struct
{
    hal_bit_t *enable;

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
} pwm_ch_shmem_t;

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
}
pwm_ch_priv_t;