/**
 ******************************************************************************
 * @file    armcncio.c
 * @author  ARMCNC site:www.armcnc.net github:armcnc.github.io
 ******************************************************************************
 */

#include "armcncio.h"

#ifdef RTAPI
MODULE_AUTHOR("armcncio");
MODULE_DESCRIPTION("Driver for ARMCNC");
MODULE_LICENSE("GPL");
#endif

static int8_t *in_pins = "";
#ifdef RTAPI
RTAPI_MP_STRING(in_pins, "channels control type, comma separated");
#endif

static int8_t *out_pins = "";
#ifdef RTAPI
RTAPI_MP_STRING(out_pins, "channels control type, comma separated");
#endif

static char *pwm_types = "";
#ifdef RTAPI
RTAPI_MP_STRING(pwm_types, "channels control type, comma separated");
#endif

static int32_t component_id;
static const uint8_t * component_name = "armcncio";

static int32_t drives_init(const char *component_name, int32_t component_id)
{
    int8_t* arg_str[2] = {in_pins, out_pins};
    int8_t n;
    uint8_t port;
    int32_t r, ch;
    int8_t *data = pwm_types, *token, type[PWM_CH_MAX_CNT] = {0};
    char name[HAL_NAME_LEN + 1];

    // init some GPIO vars
    for (n = GPIO_PINS_MAX_CNT; n--;) pin_msk[n] = 1UL << n;

    // shared memory allocation for GPIO
    for (port = GPIO_PORTS_MAX_CNT; port--;)
    {
        gpio_hal[port] = hal_malloc(GPIO_PINS_MAX_CNT * sizeof(hal_bit_t *));
        gpio_hal_not[port] = hal_malloc(GPIO_PINS_MAX_CNT * sizeof(hal_bit_t *));
        gpio_hal_pull[port] = hal_malloc(GPIO_PINS_MAX_CNT * sizeof(hal_s32_t *));
        gpio_hal_drive[port] = hal_malloc(GPIO_PINS_MAX_CNT * sizeof(hal_u32_t *));

        if (!gpio_hal[port] || !gpio_hal_not[port] || !gpio_hal_pull[port] || !gpio_hal_drive[port]) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[error]: drives_init() gpio hal_malloc failed \n");
            return -1;
        }
    }

    // export GPIO HAL pins
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

            // trying to find a correct port name
            for (found = 0, port = GPIO_PORTS_MAX_CNT; port--;) {
                if (0 == memcmp(token, gpio_name[port], 2)) {
                    found = 1;
                    break;
                }
            }

            if (!found) continue;

            // trying to find a correct pin number
            pin = (uint8_t) strtoul(&token[2], NULL, 10);

            if ((pin == 0 && token[2] != '0') || pin >= GPIO_PINS_MAX_CNT) continue;

            // export pin function
            retval = hal_pin_bit_newf((n ? HAL_IN : HAL_OUT), &gpio_hal[port][pin], component_id, "%s.gpio.%s-%s", component_name, token, type_str);

            // export pin inverted function
            retval += hal_pin_bit_newf((n ? HAL_IN : HAL_OUT), &gpio_hal_not[port][pin], component_id, "%s.gpio.%s-%s-not", component_name, token, type_str);

            // export pin pull up/down function
            retval += hal_pin_s32_newf(HAL_IN, &gpio_hal_pull[port][pin], component_id, "%s.gpio.%s-pull", component_name, token);
            
            // export pin multi-drive (open drain) function
            retval += hal_pin_u32_newf(HAL_IN, &gpio_hal_drive[port][pin], component_id, "%s.gpio.%s-multi-drive-level", component_name, token);

            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[error]: drives_init() retval failed \n");
                return -1;
            }

            // configure GPIO pin
            if (n) {
                gpio_out_cnt++;
                gpio_out_mask[port] |= pin_msk[pin];
                pinMode((int)pin, OUTPUT);
            }else{
                gpio_in_cnt++;
                gpio_in_mask[port] |= pin_msk[pin];
                pinMode((int)pin, INPUT);
            }

            // disable pull up/down
            pullUpDnControl((int)pin, PUD_OFF);

            // get/set pin init state
            *gpio_hal[port][pin] = (uint32_t)digitalRead((int)pin);
            *gpio_hal_not[port][pin] = *gpio_hal[port][pin] ? 0 : 1;
            gpio_hal_prev[port][pin] = *gpio_hal[port][pin];
            gpio_hal_not_prev[port][pin] = *gpio_hal_not[port][pin];

            // get pin pull up/down state
            switch ((uint32_t)armcnc_xj3_get_gpio_pull((char)getAlt((int)pin))) {
                case PUD_UP:      *gpio_hal_pull[port][pin] = 1;
                case PUD_DOWN:    *gpio_hal_pull[port][pin] = -1;
                default:          *gpio_hal_pull[port][pin] = 0;
            }
            gpio_hal_pull_prev[port][pin] = *gpio_hal_pull[port][pin];

            // get pin multi-drive (open drain) state
            *gpio_hal_drive[port][pin] = armcnc_xj3_get_pin_drive((char)getAlt((int)pin));
            gpio_hal_drive_prev[port][pin] = *gpio_hal_drive[port][pin];

            // used ports count update
            if (port >= gpio_ports_cnt) gpio_ports_cnt = port + 1;
            if (pin >= gpio_pins_cnt[port]) gpio_pins_cnt[port] = pin + 1;
        }
    }

    // export GPIO HAL functions
    if (gpio_out_cnt || gpio_in_cnt)
    {
        r = 0;
        rtapi_snprintf(name, sizeof(name), "%s.gpio.write", component_name);
        r += hal_export_funct(name, gpio_write, 0, 0, 0, component_id);
        rtapi_snprintf(name, sizeof(name), "%s.gpio.read", component_name);
        r += hal_export_funct(name, gpio_read, 0, 0, 0, component_id);
        if (r)
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "[error]: drives_init() gpio.write gpio.read failed \n");
            return -1;
        } 
    }

    // get PWM channels count and type
    while ((token = strtok(data, ",")) != NULL ) {
        if (data != NULL) data = NULL;
        if      (token[0] == 'P' || token[0] == 'p') type[pwm_ch_cnt++] = PWM_CTRL_BY_POS;
        else if (token[0] == 'V' || token[0] == 'v') type[pwm_ch_cnt++] = PWM_CTRL_BY_VEL;
        else if (token[0] == 'F' || token[0] == 'f') type[pwm_ch_cnt++] = PWM_CTRL_BY_FREQ;
    }

    if (pwm_ch_cnt)
    {
        // export PWM HAL functions
        r = 0;
        rtapi_snprintf(name, sizeof(name), "%s.pwm.write", component_name);
        r += hal_export_funct(name, pwm_write, 0, 1, 0, component_id);
        rtapi_snprintf(name, sizeof(name), "%s.pwm.read", component_name);
        r += hal_export_funct(name, pwm_read, 0, 1, 0, component_id);
        if (r)
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "[error]: drives_init() pwm.write pwm.read failed \n");
            return -1;
        }

        if (pwm_ch_cnt > PWM_CH_MAX_CNT) pwm_ch_cnt = PWM_CH_MAX_CNT;

        pwmh = hal_malloc(pwm_ch_cnt * sizeof(pwm_ch_hal_t));
        if (!pwmh)
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "[error]: drives_init() pwm hal_malloc failed \n");
            return -1;
        }

        #define EXPORT_PIN(IO_TYPE, VAR_TYPE, VAL, NAME, DEFAULT) \
            r += hal_pin_##VAR_TYPE##_newf(IO_TYPE, &(pwmh[ch].VAL), component_id,\
            "%s.pwm.%d." NAME, component_name, ch);\
            pwm_hal.VAL = DEFAULT;\
            pwm_private.VAL = DEFAULT;

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

            pwm_private.ctrl_type = type[ch];
            pwm_private.freq_mHz = 0;
            pwm_private.freq_min_mHz = 50000;
            pwm_private.freq_max_mHz = 500000000;
            pwm_private.dc_s32 = 0;
        }

        if (r)
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "[error]: drives_init() EXPORT_PIN failed \n");
            return -1;
        }

        #undef EXPORT_PIN
    }

    // pwm_data_set(PWM_CH_CNT, pwm_ch_cnt, 1);

    return 0;
}

static void gpio_read(void *arg, long period)
{
    static uint32_t port, pin, port_state;

    if (!gpio_in_cnt) return;

    for (port = gpio_ports_cnt; port--;)
    {
        if (!gpio_in_mask[port]) continue;

        port_state = (uint32_t)digitalRead((int)pin);

        for (pin = gpio_pins_cnt[port]; pin--;)
        {
            if (!(gpio_in_mask[port] & pin_msk[pin])) continue;

            if (port_state & pin_msk[pin]) {
                *gpio_hal[port][pin] = 1;
                *gpio_hal_not[port][pin] = 0;
            } else {
                *gpio_hal[port][pin] = 0;
                *gpio_hal_not[port][pin] = 1;
            }
        }
    }
}

static void gpio_write(void *arg, long period)
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

            // set pin pull up/down state
            if (gpio_hal_pull_prev[port][pin] != *gpio_hal_pull[port][pin])
            {
                if (*gpio_hal_pull[port][pin] > 0) {
                    *gpio_hal_pull[port][pin] = 1;
                    pullUpDnControl((int)pin, PUD_UP);
                }else if(*gpio_hal_pull[port][pin] < 0) {
                    *gpio_hal_pull[port][pin] = -1;
                    pullUpDnControl((int)pin, PUD_DOWN);
                }else{
                    pullUpDnControl((int)pin, PUD_OFF);
                }
                gpio_hal_pull_prev[port][pin] = *gpio_hal_pull[port][pin];
            }

            if (!(gpio_out_mask[port] & pin_msk[pin])) continue;

            if (*gpio_hal[port][pin] != gpio_hal_prev[port][pin])
            {
                if (*gpio_hal[port][pin]) {
                    *gpio_hal_not[port][pin] = 0;
                    mask_1 |= pin_msk[pin];
                } else {
                    *gpio_hal_not[port][pin] = 1;
                    mask_0 |= pin_msk[pin];
                }
                gpio_hal_prev[port][pin] = *gpio_hal[port][pin];
                gpio_hal_not_prev[port][pin] = *gpio_hal_not[port][pin];
            }

            if (*gpio_hal_not[port][pin] != gpio_hal_not_prev[port][pin])
            {
                if ( *gpio_hal_not[port][pin] ) {
                    *gpio_hal[port][pin] = 0;
                    mask_0 |= pin_msk[pin];
                } else {
                    *gpio_hal[port][pin] = 1;
                    mask_1 |= pin_msk[pin];
                }
                gpio_hal_not_prev[port][pin] = *gpio_hal_not[port][pin];
                gpio_hal_prev[port][pin] = *gpio_hal[port][pin];
            }

            if (mask_0) digitalWrite((int)port, LOW);
            if (mask_1) digitalWrite((int)port, HIGH);
        }
     }
}

static void pwm_read(void *arg, long period)
{
    static int32_t ch;
    for (ch = pwm_ch_cnt; ch--;)
    {
        
    }
}

static void pwm_write(void *arg, long period)
{
    static int32_t ch, dc, f;
    for (ch = pwm_ch_cnt; ch--;)
    {
        
    }
}

int rtapi_app_main(void)
{
    if (wiringPiSetup() == -1){
        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: wiringPiSetup() failed \n");
        return -1;
    }

    if ((component_id = hal_init(component_name)) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: hal_init() failed \n");
        return -1;
    }

    if (drives_init(component_name, component_id)) {
        hal_exit(component_id);
        return -1;
    }

    hal_ready(component_id);

    return 0;
}

void rtapi_app_exit(void)
{
    hal_exit(component_id);
}