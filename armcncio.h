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

#define ph *pwmh[ch]
#define pp pwmp[ch]

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

enum
{
    GPIO_PULL_DISABLE,
    GPIO_PULL_UP,
    GPIO_PULL_DOWN,
    GPIO_PULL_RESERVED3,
    GPIO_PULL_CNT
};

enum
{
    PWM_CH_POS,
    PWM_CH_TICK,
    PWM_CH_TIMEOUT,
    PWM_CH_STATE,
    PWM_CH_WATCHDOG,

    PWM_CH_P_PORT,
    PWM_CH_P_PIN_MSK,
    PWM_CH_P_PIN_MSKN,
    PWM_CH_P_INV,
    PWM_CH_P_T0,
    PWM_CH_P_T1,
    PWM_CH_P_STOP,
    PWM_CH_P_TICK,

    PWM_CH_D_PORT,
    PWM_CH_D_PIN_MSK,
    PWM_CH_D_PIN_MSKN,
    PWM_CH_D,
    PWM_CH_D_INV,
    PWM_CH_D_T0,
    PWM_CH_D_T1,
    PWM_CH_D_CHANGE,

    PWM_CH_DATA_CNT
};

enum
{
    PWM_TIMER_TICK,
    PWM_ARM_LOCK,
    PWM_ARISC_LOCK,
    PWM_CH_CNT,
    PWM_DATA_CNT
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

static const char *gpio_name[GPIO_PORTS_MAX_CNT] = {"PA","PB","PC","PD","PE","PF","PG","PL"};

static hal_bit_t **gpio_hal_0[GPIO_PORTS_MAX_CNT];
static hal_bit_t **gpio_hal_1[GPIO_PORTS_MAX_CNT];
static hal_bit_t gpio_hal_0_prev[GPIO_PORTS_MAX_CNT][GPIO_PINS_MAX_CNT];
static hal_bit_t gpio_hal_1_prev[GPIO_PORTS_MAX_CNT][GPIO_PINS_MAX_CNT];

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

static pwm_ch_shmem_t *pwmh;
static pwm_ch_priv_t pwmp[PWM_CH_MAX_CNT] = {0};
static uint8_t pwm_ch_cnt = 0;

volatile uint32_t * _pwmc[PWM_CH_MAX_CNT][PWM_CH_DATA_CNT] = {0};
volatile uint32_t * _pwmd[PWM_DATA_CNT] = {0};

static void gpio_write(void *arg, long period);
static void gpio_read(void *arg, long period);
static void pwm_write(void *arg, long period);
static void pwm_read(void *arg, long period);

static inline void _pwm_spin_lock()
{
    *_pwmd[PWM_ARM_LOCK] = 1;
    if (!*_pwmd[PWM_CH_CNT]) return;
    while ( *_pwmd[PWM_ARISC_LOCK] );
}

static void _pwm_spin_unlock()
{
    *_pwmd[PWM_ARM_LOCK] = 0;
}

static int32_t pwm_data_set(uint32_t name, uint32_t value, uint32_t safe)
{
    if (safe) {
        if (name >= PWM_DATA_CNT) return -1;
        if (name == PWM_CH_CNT && value >= PWM_CH_MAX_CNT) return -2;
    }
    _pwm_spin_lock();
    *_pwmd[name] = value;
    _pwm_spin_unlock();
    return 0;
}

static uint32_t pwm_ch_data_get(uint32_t c, uint32_t name, uint32_t safe)
{
    if (safe) {
        if (c >= PWM_CH_MAX_CNT) return 0;
        if (name >= PWM_CH_DATA_CNT) return 0;
    }
    _pwm_spin_lock();
    uint32_t value = *_pwmc[c][name];
    _pwm_spin_unlock();
    return value;
}

static int64_t pwm_ch_pos_get(uint32_t c, uint32_t safe)
{
    int64_t pos = 0, a = 0;
    int32_t pos32;
    uint32_t tc, tt;

    if (safe) {
        if (c >= PWM_CH_MAX_CNT) return 0;
    }

    _pwm_spin_lock();

    pos32 = (int32_t) *_pwmc[c][PWM_CH_POS];
    pos = (int64_t) pos32;
    pos *= 1000;
    tt = *_pwmc[c][PWM_CH_P_T0] + *_pwmc[c][PWM_CH_P_T1];

    if (tt && (*_pwmc[c][PWM_CH_STATE] == PWM_CH_STATE_P0 || *_pwmc[c][PWM_CH_STATE] == PWM_CH_STATE_P1)) {
        tc = *_pwmd[PWM_TIMER_TICK] - *_pwmc[c][PWM_CH_P_TICK];
        if (tc > tt) tc = tt;
        a = 1000 * tc / tt;
        a = *_pwmc[c][PWM_CH_D] ? 1000 - a : a - 1000;
    }

    _pwm_spin_unlock();

    return pos + a;
}

static int32_t pwm_ch_data_set(uint32_t c, uint32_t name, uint32_t value, uint32_t safe)
{
    if (safe) {
        if (c >= PWM_CH_MAX_CNT) return -1;
        if (name >= PWM_CH_DATA_CNT) return -2;
    }
    _pwm_spin_lock();
    *_pwmc[c][name] = value;
    _pwm_spin_unlock();
    return 0;
}

static inline int32_t pwm_ch_times_setup (uint32_t c, int32_t p_freq_mHz, int32_t p_duty_s32, uint32_t p_duty_max_time_ns, uint32_t d_hold_ns, uint32_t d_setup_ns, uint32_t safe) {
    uint32_t p_t0, p_t1, p_t1_max, p_period, d_t0, d_t1;
    int32_t d;

    if (safe) {
        if (c >= PWM_CH_MAX_CNT) return -1;

        if (!d_hold_ns) d_hold_ns = 50000;
        if (!d_setup_ns) d_setup_ns = 50000;

        // disable channel if it's pins wasn't setup properly
        if (*_pwmc[c][PWM_CH_P_PORT] >= GPIO_PORTS_MAX_CNT || *_pwmc[c][PWM_CH_D_PORT] >= GPIO_PORTS_MAX_CNT || !(*_pwmc[c][PWM_CH_P_PIN_MSK]) || !(*_pwmc[c][PWM_CH_D_PIN_MSK])) {
            _pwm_spin_lock();
            *_pwmc[c][PWM_CH_STATE] = PWM_CH_STATE_IDLE;
            _pwm_spin_unlock();
            return 0;
        }
    }

    // stop channel if frequency/duty_cycle == 0
    if (!p_freq_mHz || !p_duty_s32) {
        _pwm_spin_lock();
        if ( *_pwmc[c][PWM_CH_STATE] ) *_pwmc[c][PWM_CH_P_STOP] = 1;
        _pwm_spin_unlock();
        return 0;
    }

    d = (p_freq_mHz < 0 ? -1 : 1) * (p_duty_s32 < 0 ? -1 : 1);

    p_duty_s32 = p_duty_s32 < 0 ? -p_duty_s32 : p_duty_s32;
    p_freq_mHz = p_freq_mHz < 0 ? -p_freq_mHz : p_freq_mHz;

    p_period = (uint32_t) (((uint64_t)ARISC_CPU_FREQ) * ((uint64_t)1000) / ((uint64_t)p_freq_mHz));
    p_period = p_period < (2*PWM_WASTED_TICKS) ? 0 : p_period - (2*PWM_WASTED_TICKS);
    p_t1 = (uint32_t) (((uint64_t)p_period) * ((uint64_t)p_duty_s32) / ((uint64_t)INT32_MAX));

    if (p_duty_max_time_ns) {
        p_t1_max = ((uint64_t)ARISC_CPU_FREQ) * ((uint64_t)p_duty_max_time_ns) / ((uint64_t)1000000000);
        if (p_t1 > p_t1_max) p_t1 = p_t1_max;
    }

    p_t0 = p_period - p_t1;

    d_t0 = ARISC_CPU_FREQ / (1000000000 / d_hold_ns);
    d_t0 = d_t0 < PWM_WASTED_TICKS ? 0 : d_t0 - PWM_WASTED_TICKS;
    d_t1 = ARISC_CPU_FREQ / (1000000000 / d_setup_ns);
    d_t1 = d_t1 < PWM_WASTED_TICKS ? 0 : d_t1 - PWM_WASTED_TICKS;

    _pwm_spin_lock();

    *_pwmc[c][PWM_CH_D_CHANGE] = (d > 0 &&  (*_pwmc[c][PWM_CH_D])) ||
                                 (d < 0 && !(*_pwmc[c][PWM_CH_D])) ? 1 : 0;

    switch (*_pwmc[c][PWM_CH_STATE]) {
        case PWM_CH_STATE_IDLE: {*_pwmc[c][PWM_CH_STATE] = PWM_CH_STATE_P0; break;}
        case PWM_CH_STATE_P0: {*_pwmc[c][PWM_CH_TIMEOUT] = p_t0; break;}
        case PWM_CH_STATE_P1: {*_pwmc[c][PWM_CH_TIMEOUT] = p_t1; break;}
        case PWM_CH_STATE_D0: {*_pwmc[c][PWM_CH_TIMEOUT] = d_t0; break;}
        case PWM_CH_STATE_D1: {*_pwmc[c][PWM_CH_TIMEOUT] = d_t1; break;}
    }

    *_pwmc[c][PWM_CH_P_T0] = p_t0;
    *_pwmc[c][PWM_CH_P_T1] = p_t1;
    *_pwmc[c][PWM_CH_D_T0] = d_t0;
    *_pwmc[c][PWM_CH_D_T1] = d_t1;

    _pwm_spin_unlock();

    return 0;
}

static int32_t pwm_get_new_dc(uint8_t ch)
{
    if (ph.dc_cmd == pp.dc_cmd && ph.dc_scale == pp.dc_scale && ph.dc_offset == pp.dc_offset && ph.dc_min == pp.dc_min && ph.dc_max == pp.dc_max) return pp.dc_s32;
    if (ph.dc_min < -1.0) ph.dc_min = -1.0;
    if (ph.dc_max > 1.0) ph.dc_max = 1.0;
    if (ph.dc_max < ph.dc_min) ph.dc_max = ph.dc_min;
    if (ph.dc_scale < 1e-20 && ph.dc_scale > -1e-20) ph.dc_scale = 1.0;

    ph.dc_fb = ph.dc_cmd / ph.dc_scale + ph.dc_offset;

    if (ph.dc_fb < ph.dc_min) ph.dc_fb = ph.dc_min;
    if (ph.dc_fb > ph.dc_max) ph.dc_fb = ph.dc_max;

    pp.dc_cmd = ph.dc_cmd;
    pp.dc_min = ph.dc_min;
    pp.dc_max = ph.dc_max;
    pp.dc_offset = ph.dc_offset;
    pp.dc_scale = ph.dc_scale;

    return (int32_t) (ph.dc_fb * INT32_MAX);
}

static int32_t pwm_get_new_freq(uint8_t ch, long period)
{
    int32_t freq = 0;

    if (pp.freq_min != ph.freq_min) {
        pp.freq_min_mHz = (hal_u32_t) round(ph.freq_min * 1000);
        pp.freq_min = ph.freq_min;
    }
    if (pp.freq_max != ph.freq_max) {
        pp.freq_max_mHz = (hal_u32_t) round(ph.freq_max * 1000);
        pp.freq_max = ph.freq_max;
    }

    switch (pp.ctrl_type)
    {
        case PWM_CTRL_BY_POS: {
            int64_t pos_cmd_64 = (int64_t) (ph.pos_cmd * ph.pos_scale * 1000);
            int64_t pos_fdb_64 = pwm_ch_pos_get(ch,1);
            int64_t task_64 = pos_cmd_64 - pos_fdb_64;
            if (labs(task_64) < 500) freq = 0;
            else freq = (int32_t) (task_64 * ((int64_t)rtapi_clock_set_period(0)) / 1000);

            if (abs(freq) < pp.freq_min_mHz) freq = 0;
            else if (abs(freq) > pp.freq_max_mHz) freq = pp.freq_max_mHz * (freq < 0 ? -1 : 1);
            break;
        }
        case PWM_CTRL_BY_VEL: {
            if (pp.vel_cmd == ph.vel_cmd && pp.vel_scale == ph.vel_scale) {
                freq = pp.freq_mHz;
                break;
            }
            pp.vel_cmd = ph.vel_cmd;
            pp.vel_scale = ph.vel_scale;
            if (ph.vel_cmd < 1e-20 && ph.vel_cmd > -1e-20) {
                ph.vel_fb = 0;
                break;
            }
            if (ph.vel_scale < 1e-20 && ph.vel_scale > -1e-20) ph.vel_scale = 1.0;
            freq = (int32_t) round(ph.vel_scale * ph.vel_cmd * 1000);
            if (abs(freq) < pp.freq_min_mHz ) freq = 0;
            else if (abs(freq) > pp.freq_max_mHz) freq = pp.freq_max_mHz * (freq < 0 ? -1 : 1);
            ph.vel_fb = ((hal_float_t) freq) / ph.vel_scale / 1000;
            break;
        }
        case PWM_CTRL_BY_FREQ: {
            if (pp.freq_cmd == ph.freq_cmd) {
                freq = pp.freq_mHz;
                break;
            }
            pp.freq_cmd = ph.freq_cmd;
            if (ph.freq_cmd < 1e-20 && ph.freq_cmd > -1e-20) break;
            freq = (int32_t) round(ph.freq_cmd * 1000);
            if (abs(freq) < pp.freq_min_mHz) freq = 0;
            else if (abs(freq) > pp.freq_max_mHz) freq = pp.freq_max_mHz * (freq < 0 ? -1 : 1);
            break;
        }
    }

    ph.freq_fb = freq ? ((hal_float_t) freq) / 1000 : 0.0;

    return freq;
}

static int32_t pwm_ch_pins_setup(uint32_t c, uint32_t p_port, uint32_t p_pin, uint32_t p_inv, uint32_t d_port, uint32_t d_pin, uint32_t d_inv, uint32_t safe) {
    if (safe) {
        if (c >= PWM_CH_MAX_CNT) return -1;
        if (p_port >= GPIO_PORTS_MAX_CNT) return -1;
        if (p_pin >= GPIO_PINS_MAX_CNT) return -1;
        if (d_port >= GPIO_PORTS_MAX_CNT) return -1;
        if (d_pin >= GPIO_PINS_MAX_CNT) return -1;
    }

    gpio_pin_func_set(p_port, p_pin, GPIO_FUNC_OUT, safe);
    gpio_pin_pull_set(p_port, p_pin, GPIO_PULL_DISABLE, safe);
    if (p_inv){
        gpio_pin_set(p_port, p_pin, safe);
    }else{
        gpio_pin_clr(p_port, p_pin, safe);
    }

    gpio_pin_func_set(d_port, d_pin, GPIO_FUNC_OUT, safe);
    gpio_pin_pull_set(d_port, d_pin, GPIO_PULL_DISABLE, safe);
    if (d_inv){
        gpio_pin_set(d_port, d_pin, safe);
    }else{
        gpio_pin_clr(d_port, d_pin, safe);
    }

    _pwm_spin_lock();
    *_pwmc[c][PWM_CH_P_PORT] = p_port;
    *_pwmc[c][PWM_CH_P_PIN_MSK] = 1UL << p_pin;
    *_pwmc[c][PWM_CH_P_PIN_MSKN] = ~(1UL << p_pin);
    *_pwmc[c][PWM_CH_P_INV] = p_inv;
    *_pwmc[c][PWM_CH_D_PORT] = d_port;
    *_pwmc[c][PWM_CH_D_PIN_MSK] = 1UL << d_pin;
    *_pwmc[c][PWM_CH_D_PIN_MSKN] = ~(1UL << d_pin);
    *_pwmc[c][PWM_CH_D_INV] = d_inv;
    *_pwmc[c][PWM_CH_D] = 0;
    _pwm_spin_unlock();

    return 0;
}

static void pwm_pins_update(uint8_t ch)
{
    uint32_t upd = 0;

    if (pp.pwm_port != ph.pwm_port) {pp.pwm_port = ph.pwm_port; upd++;}
    if (pp.pwm_pin  != ph.pwm_pin)  {pp.pwm_pin  = ph.pwm_pin;  upd++;}
    if (pp.pwm_inv  != ph.pwm_inv)  {pp.pwm_inv  = ph.pwm_inv;  upd++;}

    if (pp.dir_port != ph.dir_port) {pp.dir_port = ph.dir_port; upd++;}
    if (pp.dir_pin != ph.dir_pin)  {pp.dir_pin  = ph.dir_pin;  upd++;}
    if (pp.dir_inv != ph.dir_inv)  {pp.dir_inv  = ph.dir_inv;  upd++;}

    if (upd) pwm_ch_pins_setup(ch, ph.pwm_port, ph.pwm_pin, ph.pwm_inv, ph.dir_port, ph.dir_pin, ph.dir_inv, 1);
}