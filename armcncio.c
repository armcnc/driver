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

static int32_t isInArray(int arr[], int size, int number)
{
    for (int i = 0; i < size; i++)
    {
        if (arr[i] == number)
        {
            return 1;
        }
    }
    return 0;
}

static int32_t hal_start(const char *component_name, int32_t component_id)
{
    int port, retval;

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

    gpio_hal_in = hal_malloc(GPIO_BCM_MAX_COUNT * sizeof(hal_bit_t *));
    gpio_hal_in_not = hal_malloc(GPIO_BCM_MAX_COUNT * sizeof(hal_bit_t *));
    gpio_hal_out = hal_malloc(GPIO_BCM_MAX_COUNT * sizeof(hal_bit_t *));
    gpio_hal_out_not = hal_malloc(GPIO_BCM_MAX_COUNT * sizeof(hal_bit_t *));
    gpio_hal_pull = hal_malloc(GPIO_BCM_MAX_COUNT * sizeof(hal_s32_t *));
    gpio_hal_drive = hal_malloc(GPIO_BCM_MAX_COUNT * sizeof(hal_u32_t *));

    if (!gpio_hal_in || !gpio_hal_in_not || !gpio_hal_out || !gpio_hal_out_not || !gpio_hal_pull || !gpio_hal_drive) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() gpio_hal failed \n");
        return -1;
    }

    char *in_pins_token = strtok(in_pins, ",");
    while (in_pins_token != NULL)
    {
        in_pins_array[in_pins_count] = atoi(in_pins_token);
        in_pins_count++;
        in_pins_token = strtok(NULL, ",");
    }

    for (int in_pins_i = 0; in_pins_i < in_pins_count; in_pins_i++)
    {
        pinMode(in_pins_array[in_pins_i], OUTPUT);

        retval = hal_pin_bit_newf(HAL_OUT, &gpio_hal_in[in_pins_array[in_pins_i]], component_id, "%s.gpio.pin%d-%s", component_name, in_pins_array[in_pins_i], "in");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() gpio_hal_in failed \n");
            return -1;
        }

        retval = hal_pin_bit_newf(HAL_OUT, &gpio_hal_in_not[in_pins_array[in_pins_i]], component_id, "%s.gpio.pin%d-%s-not", component_name, in_pins_array[in_pins_i], "in");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() gpio_hal_in_not failed \n");
            return -1;
        }

        retval = hal_pin_s32_newf(HAL_OUT, &gpio_hal_pull[in_pins_array[in_pins_i]], component_id, "%s.gpio.pin%d-%s", component_name, in_pins_array[in_pins_i], "pull");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() gpio_hal_pull failed \n");
            return -1;
        }

        retval = hal_pin_u32_newf(HAL_OUT, &gpio_hal_drive[in_pins_array[in_pins_i]], component_id, "%s.gpio.pin%d-%s", component_name, in_pins_array[in_pins_i], "drive");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() gpio_hal_drive failed \n");
            return -1;
        }

        pullUpDnControl(in_pins_array[in_pins_i], PUD_OFF);

        *gpio_hal_in[in_pins_array[in_pins_i]] = digitalRead(in_pins_array[in_pins_i]) == HIGH ? 1 : 0;
        *gpio_hal_in_not[in_pins_array[in_pins_i]] = *gpio_hal_in[in_pins_array[in_pins_i]] ? 0 : 1;
        gpio_hal_in_prev[in_pins_array[in_pins_i]] = *gpio_hal_in[in_pins_array[in_pins_i]];
        gpio_hal_in_not_prev[in_pins_array[in_pins_i]] = *gpio_hal_in_not[in_pins_array[in_pins_i]];

        switch (armcnc_xj3_get_gpio_pull((char)getAlt(in_pins_array[in_pins_i]))) {
            case PUD_UP:      *gpio_hal_pull[in_pins_array[in_pins_i]] = 1;
            case PUD_DOWN:    *gpio_hal_pull[in_pins_array[in_pins_i]] = -1;
            default:          *gpio_hal_pull[in_pins_array[in_pins_i]] = 0;
        }
        gpio_hal_pull_prev[in_pins_array[in_pins_i]] = *gpio_hal_pull[in_pins_array[in_pins_i]];

        *gpio_hal_drive[in_pins_array[in_pins_i]] = armcnc_xj3_get_pin_drive((char)getAlt(in_pins_array[in_pins_i]));
        gpio_hal_drive_prev[in_pins_array[in_pins_i]] = *gpio_hal_drive[in_pins_array[in_pins_i]];
    }

    char *out_pins_token = strtok(out_pins, ",");
    while (out_pins_token != NULL)
    {
        out_pins_array[out_pins_count] = atoi(out_pins_token);
        out_pins_count++;
        out_pins_token = strtok(NULL, ",");
    }

    for (int out_pins_i = 0; out_pins_i < out_pins_count; out_pins_i++)
    {
        pinMode(out_pins_array[out_pins_i], INPUT);

        retval = hal_pin_bit_newf(HAL_IN, &gpio_hal_out[out_pins_array[out_pins_i]], component_id, "%s.gpio.pin%d-%s", component_name, out_pins_array[out_pins_i], "out");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() gpio_hal_out failed \n");
            return -1;
        }

        retval = hal_pin_bit_newf(HAL_IN, &gpio_hal_out_not[out_pins_array[out_pins_i]], component_id, "%s.gpio.pin%d-%s-not", component_name, out_pins_array[out_pins_i], "out");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() gpio_hal_out_not failed \n");
            return -1;
        }

        *gpio_hal_out[out_pins_array[out_pins_i]] = digitalRead(out_pins_array[out_pins_i]) == HIGH ? 1 : 0;
        *gpio_hal_out_not[out_pins_array[out_pins_i]] = *gpio_hal_out[out_pins_array[out_pins_i]] ? 0 : 1;
        gpio_hal_out_prev[out_pins_array[out_pins_i]] = *gpio_hal_out[out_pins_array[out_pins_i]];
        gpio_hal_out_not_prev[out_pins_array[out_pins_i]] = *gpio_hal_out_not[out_pins_array[out_pins_i]];
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

        for (int pwm_types_i = 0; pwm_types_i < pwm_types_count; pwm_types_i++)
        {
            retval = hal_pin_bit_newf(HAL_IN, &pwm_hal[pwm_types_i].enable, component_id, "%s.pwm.%d.%s", component_name, pwm_types_i, "enable");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal enable failed \n");
                return -1;
            }
            *pwm_hal[pwm_types_i].enable = 0;

            retval = hal_pin_u32_newf(HAL_IN, &pwm_hal[pwm_types_i].pwm_port, component_id, "%s.pwm.%d.%s", component_name, pwm_types_i, "pwm-port");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal pwm_port failed \n");
                return -1;
            }
            *pwm_hal[pwm_types_i].pwm_port = UINT32_MAX;
            retval = hal_pin_u32_newf(HAL_IN, &pwm_hal[pwm_types_i].pwm_pin, component_id, "%s.pwm.%d.%s", component_name, pwm_types_i, "pwm-pin");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal pwm_pin failed \n");
                return -1;
            }
            *pwm_hal[pwm_types_i].pwm_pin = UINT32_MAX;
            retval = hal_pin_bit_newf(HAL_IN, &pwm_hal[pwm_types_i].pwm_invert, component_id, "%s.pwm.%d.%s", component_name, pwm_types_i, "pwm-invert");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal pwm_invert failed \n");
                return -1;
            }
            *pwm_hal[pwm_types_i].pwm_invert = 0;

            retval = hal_pin_u32_newf(HAL_IN, &pwm_hal[pwm_types_i].dir_port, component_id, "%s.pwm.%d.%s", component_name, pwm_types_i, "dir-port");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal dir_port failed \n");
                return -1;
            }
            *pwm_hal[pwm_types_i].dir_port = UINT32_MAX;
            retval = hal_pin_u32_newf(HAL_IN, &pwm_hal[pwm_types_i].dir_pin, component_id, "%s.pwm.%d.%s", component_name, pwm_types_i, "dir-pin");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal dir_pin failed \n");
                return -1;
            }
            *pwm_hal[pwm_types_i].dir_pin = UINT32_MAX;
            retval = hal_pin_bit_newf(HAL_IN, &pwm_hal[pwm_types_i].dir_invert, component_id, "%s.pwm.%d.%s", component_name, pwm_types_i, "dir-invert");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal dir_invert failed \n");
                return -1;
            }
            *pwm_hal[pwm_types_i].dir_invert = 0;
            retval = hal_pin_u32_newf(HAL_IO, &pwm_hal[pwm_types_i].dir_hold, component_id, "%s.pwm.%d.%s", component_name, pwm_types_i, "dir-hold");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal dir_hold failed \n");
                return -1;
            }
            *pwm_hal[pwm_types_i].dir_hold = 50000;
            retval = hal_pin_u32_newf(HAL_IO, &pwm_hal[pwm_types_i].dir_setup, component_id, "%s.pwm.%d.%s", component_name, pwm_types_i, "dir-setup");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal dir_setup failed \n");
                return -1;
            }
            *pwm_hal[pwm_types_i].dir_setup = 50000;

            retval = hal_pin_float_newf(HAL_IN, &pwm_hal[pwm_types_i].dc_cmd, component_id, "%s.pwm.%d.%s", component_name, pwm_types_i, "dc-cmd");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal dc_cmd failed \n");
                return -1;
            }
            *pwm_hal[pwm_types_i].dc_cmd = 0.0;
            retval = hal_pin_float_newf(HAL_IO, &pwm_hal[pwm_types_i].dc_min, component_id, "%s.pwm.%d.%s", component_name, pwm_types_i, "dc-min");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal dc_min failed \n");
                return -1;
            }
            *pwm_hal[pwm_types_i].dc_min = -1.0;
            retval = hal_pin_float_newf(HAL_IO, &pwm_hal[pwm_types_i].dc_max, component_id, "%s.pwm.%d.%s", component_name, pwm_types_i, "dc-max");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal dc_max failed \n");
                return -1;
            }
            *pwm_hal[pwm_types_i].dc_max = 1.0;
            retval = hal_pin_u32_newf(HAL_IO, &pwm_hal[pwm_types_i].dc_max_t, component_id, "%s.pwm.%d.%s", component_name, pwm_types_i, "dc-max-t");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal dc_max_t failed \n");
                return -1;
            }
            *pwm_hal[pwm_types_i].dc_max_t = 0;
            retval = hal_pin_float_newf(HAL_IO, &pwm_hal[pwm_types_i].dc_offset, component_id, "%s.pwm.%d.%s", component_name, pwm_types_i, "dc-offset");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal dc_offset failed \n");
                return -1;
            }
            *pwm_hal[pwm_types_i].dc_offset = 0.0;
            retval = hal_pin_float_newf(HAL_IO, &pwm_hal[pwm_types_i].dc_scale, component_id, "%s.pwm.%d.%s", component_name, pwm_types_i, "dc-scale");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal dc_scale failed \n");
                return -1;
            }
            *pwm_hal[pwm_types_i].dc_scale = 1.0;
            
            retval = hal_pin_float_newf(HAL_IO, &pwm_hal[pwm_types_i].pos_scale, component_id, "%s.pwm.%d.%s", component_name, pwm_types_i, "pos-scale");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal pos_scale failed \n");
                return -1;
            }
            *pwm_hal[pwm_types_i].pos_scale = 1.0;
            retval = hal_pin_float_newf(HAL_IN, &pwm_hal[pwm_types_i].pos_cmd, component_id, "%s.pwm.%d.%s", component_name, pwm_types_i, "pos-cmd");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal pos_cmd failed \n");
                return -1;
            }
            *pwm_hal[pwm_types_i].pos_cmd = 0.0;

            retval = hal_pin_float_newf(HAL_IO, &pwm_hal[pwm_types_i].vel_scale, component_id, "%s.pwm.%d.%s", component_name, pwm_types_i, "vel-scale");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal vel_scale failed \n");
                return -1;
            }
            *pwm_hal[pwm_types_i].vel_scale = 1.0;
            retval = hal_pin_float_newf(HAL_IN, &pwm_hal[pwm_types_i].vel_cmd, component_id, "%s.pwm.%d.%s", component_name, pwm_types_i, "vel-cmd");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal vel_cmd failed \n");
                return -1;
            }
            *pwm_hal[pwm_types_i].vel_cmd = 0.0;

            retval = hal_pin_float_newf(HAL_IO, &pwm_hal[pwm_types_i].freq_cmd, component_id, "%s.pwm.%d.%s", component_name, pwm_types_i, "freq-cmd");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal freq_cmd failed \n");
                return -1;
            }
            *pwm_hal[pwm_types_i].freq_cmd = 0.0;
            retval = hal_pin_float_newf(HAL_IO, &pwm_hal[pwm_types_i].freq_min, component_id, "%s.pwm.%d.%s", component_name, pwm_types_i, "freq-min");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal freq_min failed \n");
                return -1;
            }
            *pwm_hal[pwm_types_i].freq_min = 50.0;
            retval = hal_pin_float_newf(HAL_IO, &pwm_hal[pwm_types_i].freq_max, component_id, "%s.pwm.%d.%s", component_name, pwm_types_i, "freq-max");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal freq_max failed \n");
                return -1;
            }
            *pwm_hal[pwm_types_i].freq_max = 500000.0;

            retval = hal_pin_float_newf(HAL_OUT, &pwm_hal[pwm_types_i].dc_fb, component_id, "%s.pwm.%d.%s", component_name, pwm_types_i, "dc-fb");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal dc_fb failed \n");
                return -1;
            }
            *pwm_hal[pwm_types_i].dc_fb = 0.0;
            retval = hal_pin_float_newf(HAL_OUT, &pwm_hal[pwm_types_i].pos_fb, component_id, "%s.pwm.%d.%s", component_name, pwm_types_i, "pos-fb");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal pos_fb failed \n");
                return -1;
            }
            *pwm_hal[pwm_types_i].pos_fb = 0.0;
            retval = hal_pin_float_newf(HAL_OUT, &pwm_hal[pwm_types_i].freq_fb, component_id, "%s.pwm.%d.%s", component_name, pwm_types_i, "freq-fb");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal freq_fb failed \n");
                return -1;
            }
            *pwm_hal[pwm_types_i].freq_fb = 0.0;
            retval = hal_pin_float_newf(HAL_OUT, &pwm_hal[pwm_types_i].vel_fb, component_id, "%s.pwm.%d.%s", component_name, pwm_types_i, "vel-fb");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal vel_fb failed \n");
                return -1;
            }
            *pwm_hal[pwm_types_i].vel_fb = 0.0;
            retval = hal_pin_s32_newf(HAL_OUT, &pwm_hal[pwm_types_i].counts, component_id, "%s.pwm.%d.%s", component_name, pwm_types_i, "counts");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal vel_fb failed \n");
                return -1;
            }
            *pwm_hal[pwm_types_i].counts = 0;
        }
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
    for (int pins_i = 0; pins_i < GPIO_BCM_MAX_COUNT; pins_i++)
    {
        if (!in_pins_count || !out_pins_count) continue;
        
        if (digitalRead(in_pins_array[pins_i]) == HIGH)
        {
            *gpio_hal_in[in_pins_array[pins_i]] = 1;
            *gpio_hal_in_not[in_pins_array[pins_i]] = 0;
        }else{
            *gpio_hal_in[in_pins_array[pins_i]] = 0;
            *gpio_hal_in_not[in_pins_array[pins_i]] = 1;
        }
    }
}

static void gpio_write(void *arg, long period)
{
    for (int in_pins_i = 0; in_pins_i < in_pins_count; in_pins_i++)
    {
        if(*gpio_hal_in[in_pins_array[in_pins_i]] != gpio_hal_in_prev[in_pins_array[in_pins_i]])
        {
            if (*gpio_hal_in[in_pins_array[in_pins_i]] == HIGH)
            {
                *gpio_hal_in_not[in_pins_array[in_pins_i]] = 0;
                digitalWrite(in_pins_array[in_pins_i], HIGH);
            }else{
                *gpio_hal_in_not[in_pins_array[in_pins_i]] = 1;
                digitalWrite(in_pins_array[in_pins_i], LOW);
            }
            gpio_hal_in_prev[in_pins_array[in_pins_i]] = *gpio_hal_in[in_pins_array[in_pins_i]];
            gpio_hal_in_not_prev[in_pins_array[in_pins_i]] = *gpio_hal_in_not[in_pins_array[in_pins_i]];
        }

        if(*gpio_hal_in_not[in_pins_array[in_pins_i]] != gpio_hal_in_not_prev[in_pins_array[in_pins_i]])
        {
            if (*gpio_hal_in_not[in_pins_array[in_pins_i]] == HIGH)
            {
                *gpio_hal_in[in_pins_array[in_pins_i]] = 0;
                digitalWrite(in_pins_array[in_pins_i], LOW);
            }else{
                *gpio_hal_in[in_pins_array[in_pins_i]] = 1;
                digitalWrite(in_pins_array[in_pins_i], HIGH);
            }
            gpio_hal_in_prev[in_pins_array[in_pins_i]] = *gpio_hal_in[in_pins_array[in_pins_i]];
            gpio_hal_in_not_prev[in_pins_array[in_pins_i]] = *gpio_hal_in_not[in_pins_array[in_pins_i]];
        }

        if (*gpio_hal_pull[in_pins_array[in_pins_i]] != gpio_hal_pull_prev[in_pins_array[in_pins_i]])
        {
            if (*gpio_hal_pull[in_pins_array[in_pins_i]] == PUD_OFF)
            {
                *gpio_hal_pull[in_pins_array[in_pins_i]] = 0;
                pullUpDnControl(in_pins_array[in_pins_i], PUD_OFF);
            }
            if (*gpio_hal_pull[in_pins_array[in_pins_i]] == PUD_UP)
            {
                *gpio_hal_pull[in_pins_array[in_pins_i]] = 1;
                pullUpDnControl(in_pins_array[in_pins_i], PUD_UP);
            }
            if (*gpio_hal_pull[in_pins_array[in_pins_i]] == PUD_DOWN)
            {
                *gpio_hal_pull[in_pins_array[in_pins_i]] = -1;
                pullUpDnControl(in_pins_array[in_pins_i], PUD_DOWN);
            }
            gpio_hal_pull_prev[in_pins_array[in_pins_i]] = *gpio_hal_pull[in_pins_array[in_pins_i]];
        }
    }

    for (int out_pins_i = 0; out_pins_i < out_pins_count; out_pins_i++)
    {
        if(*gpio_hal_out[out_pins_array[out_pins_i]] != gpio_hal_out_prev[out_pins_array[out_pins_i]])
        {
            if (*gpio_hal_out[out_pins_array[out_pins_i]] == HIGH)
            {
                *gpio_hal_out_not[out_pins_array[out_pins_i]] = 0;
                digitalWrite(out_pins_array[out_pins_i], HIGH);
            }else{
                *gpio_hal_out_not[out_pins_array[out_pins_i]] = 1;
                digitalWrite(out_pins_array[out_pins_i], LOW);
            }
            gpio_hal_out_prev[out_pins_array[out_pins_i]] = *gpio_hal_out[out_pins_array[out_pins_i]];
            gpio_hal_out_not_prev[out_pins_array[out_pins_i]] = *gpio_hal_out_not[out_pins_array[out_pins_i]];
        }

        if(*gpio_hal_out_not[out_pins_array[out_pins_i]] != gpio_hal_out_not_prev[out_pins_array[out_pins_i]])
        {
            if (*gpio_hal_out_not[out_pins_array[out_pins_i]] == HIGH)
            {
                *gpio_hal_out[out_pins_array[out_pins_i]] = 0;
                digitalWrite(out_pins_array[out_pins_i], LOW);
            }else{
                *gpio_hal_out[out_pins_array[out_pins_i]] = 1;
                digitalWrite(out_pins_array[out_pins_i], HIGH);
            }
            gpio_hal_out_prev[out_pins_array[out_pins_i]] = *gpio_hal_out[out_pins_array[out_pins_i]];
            gpio_hal_out_not_prev[out_pins_array[out_pins_i]] = *gpio_hal_out_not[out_pins_array[out_pins_i]];
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