/**
 ******************************************************************************
 * @file    armcncio.c
 * @author  ARMCNC site:www.armcnc.net github:armcnc.github.io
 ******************************************************************************
 */

#include "armcncio.h"

#ifdef RTAPI
MODULE_AUTHOR("armcncio");
MODULE_DESCRIPTION("Driver for armcncio");
MODULE_LICENSE("GPL");
#endif

static int32_t comp_id;
static const uint8_t * comp_name = "armcncio";

static int8_t *in = "";
#ifdef RTAPI
RTAPI_MP_STRING(in, "input pins, comma separated");
#endif

static int8_t *out = "";
#ifdef RTAPI
RTAPI_MP_STRING(out, "output pins, comma separated");
#endif

static char *pwm = "";
#ifdef RTAPI
RTAPI_MP_STRING(pwm, "channels control type, comma separated");
#endif

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

static void gpio_write(void *arg, long period);
static void gpio_read(void *arg, long period);
static void pwm_write(void *arg, long period);
static void pwm_read(void *arg, long period);

static inline
int32_t malloc_and_export(const char *comp_name, int32_t comp_id)
{
    int8_t* arg_str[2] = {in, out};
    int8_t n;
    uint8_t port;
    int32_t r, ch;
    int8_t *data = pwm, *token, type[PWM_CH_MAX_CNT] = {0};
    char name[HAL_NAME_LEN + 1];

    for (n = GPIO_PINS_MAX_CNT; n--; ) pin_msk[n] = 1UL << n;

    for (port = GPIO_PORTS_MAX_CNT; port--;)
    {
        gpio_hal_0[port] = hal_malloc(GPIO_PINS_MAX_CNT * sizeof(hal_bit_t *));
        gpio_hal_1[port] = hal_malloc(GPIO_PINS_MAX_CNT * sizeof(hal_bit_t *));
        gpio_hal_pull[port] = hal_malloc(GPIO_PINS_MAX_CNT * sizeof(hal_s32_t *));
        gpio_hal_drive[port] = hal_malloc(GPIO_PINS_MAX_CNT * sizeof(hal_u32_t *));

        if (!gpio_hal_0[port] || !gpio_hal_1[port] || !gpio_hal_pull[port] || !gpio_hal_drive[port]) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[error]: %s.gpio: port %s hal_malloc() failed \n", comp_name, gpio_name[port]);
            return -1;
        }
    }

    for (n = 2; n--;)
    {
        if (!arg_str[n]) continue;

        int8_t *data = arg_str[n], *token;
        uint8_t pin, found;
        int32_t retval;
        int8_t* type_str = n ? "out" : "in";

        while ((token = strtok(data, ",")) != NULL)
        {
            if (data != NULL) data = NULL;
            if (strlen(token) < 3) continue;

            for (found = 0, port = GPIO_PORTS_MAX_CNT; port--;) {
                if (0 == memcmp(token, gpio_name[port], 2)) {
                    found = 1;
                    break;
                }
            }

            if (!found) continue;

            pin = (uint8_t) strtoul(&token[2], NULL, 10);

            if ((pin == 0 && token[2] != '0') || pin >= GPIO_PINS_MAX_CNT) continue;

            retval = hal_pin_bit_newf((n ? HAL_IN : HAL_OUT), &gpio_hal_0[port][pin], comp_id,"%s.gpio.%s-%s", comp_name, token, type_str);

            retval += hal_pin_bit_newf((n ? HAL_IN : HAL_OUT), &gpio_hal_1[port][pin], comp_id, "%s.gpio.%s-%s-not", comp_name, token, type_str);

            retval += hal_pin_s32_newf(HAL_IN, &gpio_hal_pull[port][pin], comp_id, "%s.gpio.%s-pull", comp_name, token);

            retval += hal_pin_u32_newf( HAL_IN, &gpio_hal_drive[port][pin], comp_id, "%s.gpio.%s-multi-drive-level", comp_name, token);

            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[error]: %s.gpio: pin %s export failed \n", comp_name, token);
                return -1;
            }

            if (n) {
                gpio_out_cnt++;
                gpio_out_mask[port] |= pin_msk[pin];
                //  gpio_pin_func_set(port, pin, GPIO_FUNC_OUT, 0);
            }else{
                gpio_in_cnt++;
                gpio_in_mask[port] |= pin_msk[pin];
                // gpio_pin_func_set(port, pin, GPIO_FUNC_IN, 0);
            }

            // gpio_pin_pull_set(port, pin, GPIO_PULL_DISABLE, 0);

            // *gpio_hal_0[port][pin] = gpio_pin_get(port, pin, 0);
            *gpio_hal_1[port][pin] = *gpio_hal_0[port][pin] ? 0 : 1;
            gpio_hal_0_prev[port][pin] = *gpio_hal_0[port][pin];
            gpio_hal_1_prev[port][pin] = *gpio_hal_1[port][pin];

            switch (gpio_pin_pull_get(port, pin, 0)) {
                case GPIO_PULL_UP:      *gpio_hal_pull[port][pin] = 1;
                case GPIO_PULL_DOWN:    *gpio_hal_pull[port][pin] = -1;
                default:                *gpio_hal_pull[port][pin] = 0;
            }

            gpio_hal_pull_prev[port][pin] = *gpio_hal_pull[port][pin];

            // *gpio_hal_drive[port][pin] = gpio_pin_multi_drive_get(port, pin, 0);
            gpio_hal_drive_prev[port][pin] = *gpio_hal_drive[port][pin];

            if (port >= gpio_ports_cnt) gpio_ports_cnt = port + 1;

            if (pin >= gpio_pins_cnt[port]) gpio_pins_cnt[port] = pin + 1;
        }
    }

    if (gpio_out_cnt || gpio_in_cnt) {
         r = 0;
        rtapi_snprintf(name, sizeof(name), "%s.gpio.write", comp_name);
        r += hal_export_funct(name, gpio_write, 0, 0, 0, comp_id);
        rtapi_snprintf(name, sizeof(name), "%s.gpio.read", comp_name);
        r += hal_export_funct(name, gpio_read, 0, 0, 0, comp_id);
        if (r) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[error]: %s.gpio: HAL functions export failed\n", comp_name);
            return -1;
        }
    }

    while ( (token = strtok(data, ",")) != NULL ) {
        if ( data != NULL ) data = NULL;
        if      ( token[0] == 'P' || token[0] == 'p' ) type[pwm_ch_cnt++] = PWM_CTRL_BY_POS;
        else if ( token[0] == 'V' || token[0] == 'v' ) type[pwm_ch_cnt++] = PWM_CTRL_BY_VEL;
        else if ( token[0] == 'F' || token[0] == 'f' ) type[pwm_ch_cnt++] = PWM_CTRL_BY_FREQ;
    }

    if (pwm_ch_cnt)
    {
        r = 0;
        rtapi_snprintf(name, sizeof(name), "%s.pwm.write", comp_name);
        r += hal_export_funct(name, pwm_write, 0, 1, 0, comp_id);
        rtapi_snprintf(name, sizeof(name), "%s.pwm.read", comp_name);
        r += hal_export_funct(name, pwm_read, 0, 1, 0, comp_id);
        if (r) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[error]: %s.pwm: HAL functions export failed\n", comp_name);
            return -1;
        }

        if (pwm_ch_cnt > PWM_CH_MAX_CNT) pwm_ch_cnt = PWM_CH_MAX_CNT;

         pwmh = hal_malloc(pwm_ch_cnt * sizeof(pwm_ch_shmem_t));
         if (!pwmh) PRINT_ERROR_AND_RETURN("[error]: hal_malloc() failed", -1);

         for (r = 0, ch = pwm_ch_cnt; ch--;)
         {
            EXPORT_PIN(HAL_IN, bit, enable, "enable", 0);
            EXPORT_PIN(HAL_IN, u32, pwm_port, "pwm-port", UINT32_MAX);
            EXPORT_PIN(HAL_IN, u32, pwm_pin, "pwm-pin", UINT32_MAX);
            EXPORT_PIN(HAL_IN, bit, pwm_inv, "pwm-invert", 0);

            EXPORT_PIN(HAL_IN, u32, dir_port, "dir-port", UINT32_MAX);
            EXPORT_PIN(HAL_IN, u32, dir_pin, "dir-pin", UINT32_MAX);
            EXPORT_PIN(HAL_IN, bit, dir_inv, "dir-invert", 0);
            EXPORT_PIN(HAL_IO, u32, dir_hold, "dir-hold", 50000);
            EXPORT_PIN(HAL_IO, u32, dir_setup, "dir-setup", 50000);

            EXPORT_PIN(HAL_IN, float, dc_cmd, "dc-cmd", 0.0);
            EXPORT_PIN(HAL_IO, float, dc_min, "dc-min", -1.0);
            EXPORT_PIN(HAL_IO, float, dc_max, "dc-max", 1.0);
            EXPORT_PIN(HAL_IO, u32, dc_max_t, "dc-max-t", 0);
            EXPORT_PIN(HAL_IO, float, dc_offset, "dc-offset", 0.0);
            EXPORT_PIN(HAL_IO, float, dc_scale, "dc-scale", 1.0);

            EXPORT_PIN(HAL_IO, float, pos_scale, "pos-scale", 1.0);
            EXPORT_PIN(HAL_IN, float, pos_cmd, "pos-cmd", 0.0);

            EXPORT_PIN(HAL_IO, float, vel_scale, "vel-scale", 1.0);
            EXPORT_PIN(HAL_IN, float, vel_cmd, "vel-cmd", 0.0);

            EXPORT_PIN(HAL_IO, float, freq_cmd, "freq-cmd", 0.0);
            EXPORT_PIN(HAL_IO, float, freq_min, "freq-min", 50.0);
            EXPORT_PIN(HAL_IO, float, freq_max, "freq-max", 500000.0);

            EXPORT_PIN(HAL_OUT, float, dc_fb, "dc-fb", 0.0);
            EXPORT_PIN(HAL_OUT, float, pos_fb, "pos-fb", 0.0);
            EXPORT_PIN(HAL_OUT, float, freq_fb, "freq-fb", 0.0);
            EXPORT_PIN(HAL_OUT, float, vel_fb, "vel-fb", 0.0);
            EXPORT_PIN(HAL_OUT, s32, counts, "counts", 0);

            pp.ctrl_type = type[ch];
            pp.freq_mHz = 0;
            pp.freq_min_mHz = 50000;
            pp.freq_max_mHz = 500000000;
            pp.dc_s32 = 0;
         }
         if ( r ) {
             rtapi_print_msg(RTAPI_MSG_ERR, "[error]: %s.pwm: HAL pins export failed\n", comp_name);
             return -1;
         }

         #undef EXPORT_PIN
    }

    // pwm_data_set(PWM_CH_CNT, pwm_ch_cnt, 1);

    return 0;
}

static inline
void gpio_read(void *arg, long period)
{
    static uint32_t port, pin, port_state;

    if ( !gpio_in_cnt ) return;

    for (port = gpio_ports_cnt; port--;)
    {
        if (!gpio_in_mask[port]) continue;

        // port_state = gpio_port_get(port, 0);

        for (pin = gpio_pins_cnt[port]; pin--;) {
            if (!(gpio_in_mask[port] & pin_msk[pin])) continue;

            if ( port_state & pin_msk[pin] ) {
                *gpio_hal_0[port][pin] = 1;
                *gpio_hal_1[port][pin] = 0;
            } else {
                *gpio_hal_0[port][pin] = 0;
                *gpio_hal_1[port][pin] = 1;
            }
        }
    }
}

static inline
void gpio_write(void *arg, long period)
{
    static uint32_t port, pin, mask_0, mask_1;

    if (!gpio_in_cnt && !gpio_out_cnt) return;

    for (port = gpio_ports_cnt; port--;)
    {
        if (!gpio_in_mask[port] && !gpio_out_mask[port]) continue;

        mask_0 = 0;
        mask_1 = 0;

        for (pin = gpio_pins_cnt[port]; pin--;)
        {
            if (!(gpio_in_mask[port] & pin_msk[pin]) && !(gpio_out_mask[port] & pin_msk[pin])) continue;

            if (gpio_hal_pull_prev[port][pin] != *gpio_hal_pull[port][pin])
            {
                if (*gpio_hal_pull[port][pin] > 0) {
                    *gpio_hal_pull[port][pin] = 1;
                    // gpio_pin_pull_set(port, pin, GPIO_PULL_UP, 0);
                }else if (*gpio_hal_pull[port][pin] < 0) {
                    *gpio_hal_pull[port][pin] = -1;
                    // gpio_pin_pull_set(port, pin, GPIO_PULL_DOWN, 0);
                }else{
                    // gpio_pin_pull_set(port, pin, GPIO_PULL_DISABLE, 0);
                }
                gpio_hal_pull_prev[port][pin] = *gpio_hal_pull[port][pin];
            }

            if (gpio_hal_drive_prev[port][pin] != *gpio_hal_drive[port][pin]) {
                *gpio_hal_drive[port][pin] &= (GPIO_PULL_CNT - 1);
                // gpio_pin_multi_drive_set(port, pin, *gpio_hal_drive[port][pin], 0);
                gpio_hal_drive_prev[port][pin] = *gpio_hal_drive[port][pin];
            }

            if (!(gpio_out_mask[port] & pin_msk[pin])) continue;

            if ( *gpio_hal_0[port][pin] != gpio_hal_0_prev[port][pin]) {
                if (*gpio_hal_0[port][pin]) {
                    *gpio_hal_1[port][pin] = 0;
                    mask_1 |= pin_msk[pin];
                } else {
                    *gpio_hal_1[port][pin] = 1;
                    mask_0 |= pin_msk[pin];
                }
                gpio_hal_0_prev[port][pin] = *gpio_hal_0[port][pin];
                gpio_hal_1_prev[port][pin] = *gpio_hal_1[port][pin];
            }

            if ( *gpio_hal_1[port][pin] != gpio_hal_1_prev[port][pin]) {
                if (*gpio_hal_1[port][pin]) {
                    *gpio_hal_0[port][pin] = 0;
                    mask_0 |= pin_msk[pin];
                } else {
                    *gpio_hal_0[port][pin] = 1;
                    mask_1 |= pin_msk[pin];
                }
                gpio_hal_1_prev[port][pin] = *gpio_hal_1[port][pin];
                gpio_hal_0_prev[port][pin] = *gpio_hal_0[port][pin];
            }
        }

        // if (mask_0) gpio_port_clr(port, mask_0, 0);
        // if (mask_1) gpio_port_set(port, mask_1, 0);
    }
}

static void pwm_read(void *arg, long period)
{
    static int32_t ch;

    for (ch = pwm_ch_cnt; ch--;) {
        if (!ph.enable) continue;

        if (ph.pos_scale < 1e-20 && ph.pos_scale > -1e-20) ph.pos_scale = 1.0;

        if (pp.ctrl_type == PWM_CTRL_BY_POS) {
            ph.counts = (int32_t) pwm_ch_data_get(ch, PWM_CH_POS, 1);
            pp.counts = (int32_t) (ph.pos_scale * ph.pos_cmd);
            if (abs(pp.counts - ph.counts) < 10) {
                ph.counts = pp.counts;
                ph.pos_fb = ph.pos_cmd;
            }else{
                ph.pos_fb = ((hal_float_t)pwm_ch_pos_get(ch,1)) / ph.pos_scale / 1000;
            }
        } else {
            ph.pos_fb = ((hal_float_t)pwm_ch_pos_get(ch,1)) / ph.pos_scale / 1000;
        }
    }
}

static void pwm_write(void *arg, long period)
{
    static int32_t ch, dc, f;

    for (ch = pwm_ch_cnt; ch--;) {
        if (pp.enable != ph.enable) {
            pp.enable = ph.enable;
            if ( !ph.enable ) {
                // pwm_ch_data_set(ch, PWM_CH_WATCHDOG, 0, 1);
                // pwm_ch_times_setup(ch, 0, 0, ph.dc_max_t, ph.dir_hold, ph.dir_setup, 1);
                continue;
            }
        }
        // pwm_pins_update(ch);
        // dc = pwm_get_new_dc(ch);
        // f = pwm_get_new_freq(ch, period);
        // pwm_ch_data_set(ch, PWM_CH_WATCHDOG, (f && dc ? 1000 : 0), 1);
        if (pp.freq_mHz != f || pp.dc_s32 != dc) {
            // pwm_ch_times_setup(ch, f, dc, ph.dc_max_t, ph.dir_hold, ph.dir_setup, 1);
            pp.dc_s32 = dc;
            pp.freq_mHz = f;
        }
    }
}

int32_t rtapi_app_main(void)
{
    if ((comp_id = hal_init(comp_name)) < 0) {
        PRINT_ERROR_AND_RETURN("[error]: hal_init() failed\n", -1);
    }

    if (malloc_and_export(comp_name, comp_id)) {
        hal_exit(comp_id);
        return -1;
    }

    hal_ready(comp_id);

    return 0;
}

void rtapi_app_exit(void)
{
    hal_exit(comp_id);
}