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

static char *in_pins = "";
#ifdef RTAPI
RTAPI_MP_STRING(in_pins, "channels control type, comma separated");
#endif

static char *out_pins = "";
#ifdef RTAPI
RTAPI_MP_STRING(out_pins, "channels control type, comma separated");
#endif

static char *pwm_types = "";
#ifdef RTAPI
RTAPI_MP_STRING(pwm_types, "channels control type, comma separated");
#endif

static int32_t component_id;
static const uint8_t * component_name = "armcncio";

static int32_t hal_start(const char *component_name, int32_t component_id)
{
    int retval;

    char name[HAL_NAME_LEN + 1];

    if (in_pins == NULL || in_pins[0] == '\0')
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() in_pins failed \n");
        return -1;
    }

    if (out_pins == NULL || out_pins[0] == '\0')
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() out_pins failed \n");
        return -1;
    }

    if (pwm_types == NULL || pwm_types[0] == '\0')
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_types failed \n");
        return -1;
    }

    gpio_hal = hal_malloc(GPIO_BCM_MAX_COUNT * sizeof(hal_bit_t *));
    gpio_hal_not = hal_malloc(GPIO_BCM_MAX_COUNT * sizeof(hal_bit_t *));

    if (!gpio_hal || !gpio_hal_not) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() gpio_hal failed \n");
        return -1;
    }

    for (int n = 0; n < GPIO_BCM_MAX_COUNT; n++) gpio_mask[n] = 1UL << n;

    char *in_pins_token = strtok(in_pins, ",");
    while (in_pins_token != NULL)
    {
        in_pins_array[in_pins_count] = atoi(in_pins_token);
        in_pins_count++;
        in_pins_token = strtok(NULL, ",");
    }

    char *out_pins_token = strtok(out_pins, ",");
    while (out_pins_token != NULL)
    {
        out_pins_array[out_pins_count] = atoi(out_pins_token);
        out_pins_count++;
        out_pins_token = strtok(NULL, ",");
    }

    for (int in_pins_i = 0; in_pins_i < in_pins_count; in_pins_i++)
    {
        pinMode(in_pins_array[in_pins_i], INPUT);

        gpio_in_mask[in_pins_array[in_pins_i]] |= gpio_mask[in_pins_array[in_pins_i]];

        retval = hal_pin_bit_newf(HAL_OUT, &gpio_hal[in_pins_array[in_pins_i]], component_id, "%s.gpio.pin%d-%s", component_name, in_pins_array[in_pins_i], "in");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() gpio_hal_in failed \n");
            return -1;
        }

        retval = hal_pin_bit_newf(HAL_OUT, &gpio_hal_not[in_pins_array[in_pins_i]], component_id, "%s.gpio.pin%d-%s-not", component_name, in_pins_array[in_pins_i], "in");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() gpio_hal_in_not failed \n");
            return -1;
        }

        pullUpDnControl(in_pins_array[in_pins_i], PUD_OFF);

        *gpio_hal[in_pins_array[in_pins_i]] = digitalRead(in_pins_array[in_pins_i]) == HIGH ? 1 : 0;
        *gpio_hal_not[in_pins_array[in_pins_i]] = *gpio_hal[in_pins_array[in_pins_i]] ? 0 : 1;
        gpio_hal_prev[in_pins_array[in_pins_i]] = *gpio_hal[in_pins_array[in_pins_i]];
        gpio_hal_not_prev[in_pins_array[in_pins_i]] = *gpio_hal_not[in_pins_array[in_pins_i]];
    }

    for (int out_pins_i = 0; out_pins_i < out_pins_count; out_pins_i++)
    {
        pinMode(out_pins_array[out_pins_i], OUTPUT);

        gpio_out_mask[out_pins_array[out_pins_i]] |= gpio_mask[out_pins_array[out_pins_i]];

        retval = hal_pin_bit_newf(HAL_IN, &gpio_hal[out_pins_array[out_pins_i]], component_id, "%s.gpio.pin%d-%s", component_name, out_pins_array[out_pins_i], "out");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() gpio_hal_out failed \n");
            return -1;
        }

        retval = hal_pin_bit_newf(HAL_IN, &gpio_hal_not[out_pins_array[out_pins_i]], component_id, "%s.gpio.pin%d-%s-not", component_name, out_pins_array[out_pins_i], "out");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() gpio_hal_out_not failed \n");
            return -1;
        }

        pullUpDnControl(out_pins_array[out_pins_i], PUD_OFF);

        *gpio_hal[out_pins_array[out_pins_i]] = digitalRead(out_pins_array[out_pins_i]) == HIGH ? 1 : 0;
        *gpio_hal_not[out_pins_array[out_pins_i]] = *gpio_hal[out_pins_array[out_pins_i]] ? 0 : 1;
        gpio_hal_prev[out_pins_array[out_pins_i]] = *gpio_hal[out_pins_array[out_pins_i]];
        gpio_hal_not_prev[out_pins_array[out_pins_i]] = *gpio_hal_not[out_pins_array[out_pins_i]];
    }

    char *pwm_types_token = strtok(pwm_types, ",");
    while (pwm_types_token != NULL)
    {
        pwm_types_array[pwm_types_count] = (strcmp(pwm_types_token, "p") == 0) ? 1 : 2;
        pwm_types_count++;
        pwm_types_token = strtok(NULL, ",");
    }

    if (pwm_types_count > 0)
    {
        pwm_hal = hal_malloc(pwm_types_count * sizeof(pwm_hal_struct));
        if (!pwm_hal) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal failed \n");
            return -1;
        }

        retval = 0;
        #define EXPORT_PIN(IO_TYPE,VAR_TYPE,VAL,NAME,DEFAULT) \
            retval += hal_pin_##VAR_TYPE##_newf(IO_TYPE, &(pwm_hal[ch].VAL), component_id,\
            "%s.pwm.%d." NAME, component_name, ch);\
            pwm_hal_var.VAL = DEFAULT;\
            pwm_private_var.VAL = DEFAULT;

        for (int ch = 0; ch < pwm_types_count; ch++)
        {
            EXPORT_PIN(HAL_IN, bit, enable, "enable", 0);

            EXPORT_PIN(HAL_IN, u32, pwm_port, "pwm-port", UINT32_MAX);
            EXPORT_PIN(HAL_IN, u32, pwm_pin, "pwm-pin", UINT32_MAX);
            EXPORT_PIN(HAL_IN, bit, pwm_invert, "pwm-invert", 0);

            EXPORT_PIN(HAL_IN, u32, dir_port ,"dir-port", UINT32_MAX);
            EXPORT_PIN(HAL_IN, u32, dir_pin, "dir-pin", UINT32_MAX);
            EXPORT_PIN(HAL_IN, bit, dir_invert, "dir-invert", 0);
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

            pwm_private_var.ctrl_type = pwm_types_array[ch];
            pwm_private_var.freq_mHz = 0;
            pwm_private_var.freq_min_mHz = 50000;
            pwm_private_var.freq_max_mHz = 500000000;
            pwm_private_var.dc_s32 = 0;
        }

        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() EXPORT_PIN failed \n");
            return -1;
        }

        #undef EXPORT_PIN
    }

    rtapi_snprintf(name, sizeof(name), "%s.gpio.write", component_name);
    retval = hal_export_funct(name, gpio_write, 0, 0, 0, component_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() gpio_write failed \n");
        return -1;
    }

    rtapi_snprintf(name, sizeof(name), "%s.gpio.read", component_name);
    retval = hal_export_funct(name, gpio_read, 0, 0, 0, component_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() gpio_read failed \n");
        return -1;
    }

    rtapi_snprintf(name, sizeof(name), "%s.pwm.write", component_name);
    retval = hal_export_funct(name, pwm_write, 0, 1, 0, component_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_write failed \n");
        return -1;
    }

    rtapi_snprintf(name, sizeof(name), "%s.pwm.read", component_name);
    retval = hal_export_funct(name, pwm_read, 0, 1, 0, component_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_read failed \n");
        return -1;
    }

    return 0;
}

static void gpio_read(void *arg, long period)
{
    if (!in_pins_count) return;

    for (int pin = 0; pin < GPIO_BCM_MAX_COUNT; pin++)
    {
        if (!gpio_in_mask[pin]) continue;

        if (!(gpio_in_mask[pin] & gpio_mask[pin])) continue;

        uint32_t pin_state = digitalRead(pin) == HIGH ? gpio_mask[pin] : 0;

        if (pin_state & gpio_mask[pin])
        {
            *gpio_hal[pin] = 1;
            *gpio_hal_not[pin] = 0;
        }else{
            *gpio_hal[pin] = 0;
            *gpio_hal_not[pin] = 1;
        }
    }
}

static void gpio_write(void *arg, long period)
{
    static uint32_t mask_0, mask_1;

    if (!in_pins_count && !out_pins_count) return;

    for (int pin = 0; pin < GPIO_BCM_MAX_COUNT; pin++)
    {
        if (!gpio_in_mask[pin] && !gpio_out_mask[pin]) continue;

        mask_0 = 0;
        mask_1 = 0;

        if (!(gpio_in_mask[pin] & gpio_mask[pin]) && !(gpio_out_mask[pin] & gpio_mask[pin])) continue;

        if (!(gpio_out_mask[pin] & gpio_mask[pin])) continue;

        if(*gpio_hal[pin] != gpio_hal_prev[pin])
        {
            if (*gpio_hal[pin] == HIGH)
            {
                *gpio_hal_not[pin] = 0;
                mask_1 |= gpio_mask[pin];
            }else{
                *gpio_hal_not[pin] = 1;
                mask_0 |= gpio_mask[pin];
            }

            gpio_hal_prev[pin] = *gpio_hal[pin];
            gpio_hal_not_prev[pin] = *gpio_hal_not[pin];
        }

        if(*gpio_hal_not[pin] != gpio_hal_not_prev[pin])
        {
            if (*gpio_hal_not[pin] == HIGH)
            {
                *gpio_hal[pin] = 0;
                mask_0 |= gpio_mask[pin];
            }else{
                *gpio_hal[pin] = 1;
                mask_1 |= gpio_mask[pin];
            }

            gpio_hal_not_prev[pin] = *gpio_hal_not[pin];
            gpio_hal_prev[pin] = *gpio_hal[pin];
        }

        if (mask_0) digitalWrite(pin, LOW);
        if (mask_1) digitalWrite(pin, HIGH);
    }
}

static void pwm_read(void *arg, long period)
{
    for (int ch = 0; ch < pwm_types_count; ch++)
    {
        if (!pwm_hal_var.enable) continue;
    }
}

static void pwm_write(void *arg, long period)
{
    for (int ch = 0; ch < pwm_types_count; ch++)
    {
        if (pwm_private_var.enable != pwm_hal_var.enable)
        {
            pwm_private_var.enable = pwm_hal_var.enable;
            if (!pwm_hal_var.enable)
            {
                rtapi_print_msg(RTAPI_MSG_DBG, "enable \n");
                softPwmWrite((int)pwm_hal_var.pwm_pin, 0);
                softPwmStop((int)pwm_hal_var.pwm_pin);
                continue;
            }
        }

        pwm_pins_update(ch);

        // int32_t dc = pwm_get_new_dc(ch);

        // float sValue = pwm_hal_var.dc_cmd;
        // float dcScale = pwm_hal_var.dc_scale / 2;
        // int pwmValue = (int)(500.0 * sValue / dcScale);
        // pwmValue = pwmValue < 0 ? 0 : (pwmValue > 500 ? 500 : pwmValue);
        softPwmWrite(pwm_hal_var.pwm_pin, 200);
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

    if (hal_start(component_name, component_id)) {
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