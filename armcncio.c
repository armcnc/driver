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
    int8_t *data = pwm, *token, type[PWM_CH_MAX_CNT] = {0};
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
            rtapi_print_msg(RTAPI_MSG_ERR, "[error]: drives_init() hal_malloc failed \n");
            return -1;
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

                if (strlen(token) < 4) continue;

                // trying to find a correct port name
                for (found = 0, port = GPIO_PORTS_MAX_CNT; port--;) {
                    if (0 == memcmp(token, gpio_name[port], 3)) {
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
                    pinMode(pin, OUTPUT);
                }else{
                    gpio_in_cnt++;
                    gpio_in_mask[port] |= pin_msk[pin];
                    pinMode(pin, INPUT);
                }

                // disable pull up/down
                pullUpDnControl(pin, PUD_OFF);

                // get/set pin init state
                *gpio_hal[port][pin] = digitalRead(pin, 0);
                *gpio_hal_not[port][pin] = *gpio_hal[port][pin] ? 0 : 1;
                gpio_hal_prev[port][pin] = *gpio_hal[port][pin];
                gpio_hal_not_prev[port][pin] = *gpio_hal_not[port][pin];

                // get pin pull up/down state
                switch (armcnc_xj3_get_gpio_pull(getAlt(pin))) {
                    case PUD_UP:      *gpio_hal_pull[port][pin] = 1;
                    case PUD_DOWN:    *gpio_hal_pull[port][pin] = -1;
                    default:          *gpio_hal_pull[port][pin] = 0;
                }
                gpio_hal_pull_prev[port][pin] = *gpio_hal_pull[port][pin];

                // get pin multi-drive (open drain) state
                *gpio_hal_drive[port][pin] = armcnc_xj3_get_pin_drive(getAlt(pin));
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
    }

    return 0;
}

static void gpio_read(void *arg, long period)
{
    static uint32_t port, pin, port_state;

    if (!gpio_in_cnt) return;

    for (port = gpio_ports_cnt; port--;)
    {
        if (!gpio_in_mask[port]) continue;

        port_state = digitalRead(port);

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
                    pullUpDnControl(pin, PUD_UP);
                }else if(*gpio_hal_pull[port][pin] < 0) {
                    *gpio_hal_pull[port][pin] = -1;
                    pullUpDnControl(pin, PUD_DOWN);
                }else{
                    pullUpDnControl(pin, PUD_OFF);
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

            if (mask_0) digitalWrite(port, LOW);
            if (mask_1) digitalWrite(port, HIGH);
        }
     }
}

static void pwm_read(void *arg, long period)
{
    for (int pwm_types_i = 0; pwm_types_i < pwm_types_count; pwm_types_i++)
    {
        
    }
}

static void pwm_write(void *arg, long period)
{
    for (int pwm_types_i = 0; pwm_types_i < pwm_types_count; pwm_types_i++)
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