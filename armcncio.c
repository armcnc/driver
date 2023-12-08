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

static int8_t *armcnc_in = "";
#ifdef RTAPI
RTAPI_MP_STRING(armcnc_in, "input pins, comma separated");
#endif

static int8_t *armcnc_out = "";
#ifdef RTAPI
RTAPI_MP_STRING(armcnc_out, "output pins, comma separated");
#endif

static char *armcnc_pwm = "";
#ifdef RTAPI
RTAPI_MP_STRING(armcnc_pwm, "channels control type, comma separated");
#endif

static int32_t malloc_and_export(const char *comp_name, int32_t comp_id)
{
    int8_t* arg_str[2] = {armcnc_in, armcnc_out};
    int8_t n;
    uint8_t port;
    int32_t r, ch;
    int8_t *data = armcnc_pwm, *token, type[PWM_CH_MAX_CNT] = {0};
    char name[HAL_NAME_LEN + 1];

    for (n = GPIO_PINS_MAX_CNT; n--; ) pin_msk[n] = 1UL << n;

    for (port = GPIO_PORTS_MAX_CNT; port--;)
    {
        gpio_hal_0[port] = hal_malloc(GPIO_PINS_MAX_CNT * sizeof(hal_bit_t *));
        gpio_hal_1[port] = hal_malloc(GPIO_PINS_MAX_CNT * sizeof(hal_bit_t *));
        gpio_hal_pull[port] = hal_malloc(GPIO_PINS_MAX_CNT * sizeof(hal_s32_t *));
        gpio_hal_drive[port] = hal_malloc(GPIO_PINS_MAX_CNT * sizeof(hal_u32_t *));

        if (!gpio_hal_0[port] || !gpio_hal_1[port] || !gpio_hal_pull[port] || !gpio_hal_drive[port]) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[error]: %s.gpio: port %s hal_malloc() failed\n", comp_name, gpio_name[port]);
            return -1;
        }
    }

    for (n = 2; n--;)
    {
        if (!arg_str[n]) continue;

        int8_t *data = arg_str[n], *token;
        int pin;
        int32_t retval;
        int8_t* type_str = n ? "out" : "in";

        while ((token = strtok(data, ",")) != NULL)
        {
            if (data != NULL) data = NULL;

            pin = atoi(token);

            if (pin == 0 || pin >= GPIO_PINS_MAX_CNT) continue;

            retval = hal_pin_bit_newf((n ? HAL_IN : HAL_OUT), &gpio_hal_0[port][pin], comp_id,"%s.gpio.%s-%s", comp_name, token, type_str);

            retval += hal_pin_bit_newf((n ? HAL_IN : HAL_OUT), &gpio_hal_1[port][pin], comp_id, "%s.gpio.%s-%s-not", comp_name, token, type_str);

            retval += hal_pin_s32_newf(HAL_IN, &gpio_hal_pull[port][pin], comp_id, "%s.gpio.%s-pull", comp_name, token);

            retval += hal_pin_u32_newf(HAL_IN, &gpio_hal_drive[port][pin], comp_id, "%s.gpio.%s-multi-drive-level", comp_name, token);

            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[error]: %s.gpio: pin %s export failed\n", comp_name, token);
                return -1;
            }

            if (n) {
                gpio_out_cnt++;
                gpio_out_mask[port] |= pin_msk[pin];
                pinMode(pin, OUTPUT);
            }else{
                gpio_in_cnt++;
                gpio_in_mask[port] |= pin_msk[pin];
                pinMode(pin, INPUT);
            }

            pullUpDnControl(pin, PUD_OFF);

            *gpio_hal_0[port][pin] = digitalRead(pin);
            *gpio_hal_1[port][pin] = *gpio_hal_0[port][pin] ? 0 : 1;
            gpio_hal_0_prev[port][pin] = *gpio_hal_0[port][pin];
            gpio_hal_1_prev[port][pin] = *gpio_hal_1[port][pin];

            switch (xj3_get_gpio_pull(getAlt(pin))) {
                case PUD_UP:      *gpio_hal_pull[port][pin] = 2;
                case PUD_DOWN:    *gpio_hal_pull[port][pin] = 1;
                default:          *gpio_hal_pull[port][pin] = 0;
            }

            gpio_hal_pull_prev[port][pin] = *gpio_hal_pull[port][pin];

            *gpio_hal_drive[port][pin] = xj3_get_pin_drive(getAlt(pin));
            gpio_hal_drive_prev[port][pin] = *gpio_hal_drive[port][pin];

            if (port >= gpio_ports_cnt) gpio_ports_cnt = port + 1;

            if (pin >= gpio_pins_cnt[port]) gpio_pins_cnt[port] = pin + 1;
        }
    }

    return 0;
}

static void gpio_read(void *arg, long period)
{
    static uint32_t port, pin;

    if (!gpio_in_cnt) return;

    for (port = gpio_ports_cnt; port--;)
    {
        if (!gpio_in_mask[port]) continue;

        for (pin = gpio_pins_cnt[port]; pin--;) {
            if (!(gpio_in_mask[port] & pin_msk[pin])) continue;

            int port_state = digitalRead(pin);

            if (port_state & pin_msk[pin]) {
                *gpio_hal_0[port][pin] = 1;
                *gpio_hal_1[port][pin] = 0;
            } else {
                *gpio_hal_0[port][pin] = 0;
                *gpio_hal_1[port][pin] = 1;
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

            if (gpio_hal_pull_prev[port][pin] != *gpio_hal_pull[port][pin])
            {
                if (*gpio_hal_pull[port][pin] == 2) {
                    *gpio_hal_pull[port][pin] = 2;
                    pullUpDnControl(pin, PUD_UP);
                }else if (*gpio_hal_pull[port][pin] == 1) {
                    *gpio_hal_pull[port][pin] = 1;
                    pullUpDnControl(pin, PUD_DOWN);
                }else{
                    *gpio_hal_pull[port][pin] = 0;
                    pullUpDnControl(pin, PUD_OFF);
                }
                gpio_hal_pull_prev[port][pin] = *gpio_hal_pull[port][pin];
            }

            if (gpio_hal_drive_prev[port][pin] != *gpio_hal_drive[port][pin]) {
                *gpio_hal_drive[port][pin] &= (GPIO_PULL_CNT - 1);
                xj3_set_pin_drive(getAlt(pin), *gpio_hal_drive[port][pin]);
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

        if (mask_0 & pin_msk[pin]) {
            digitalWrite(pin_msk[pin], LOW);
        }

        if (mask_1 & pin_msk[pin]) {
            digitalWrite(pin_msk[pin], HIGH);
        }
    }
}

int rtapi_app_main(void)
{
    if ((comp_id = hal_init(comp_name)) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: hal_init() failed\n");
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