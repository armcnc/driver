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

static int32_t hal_malloc_init(void)
{
    gpio_in_out = hal_malloc(GPIO_BCM_MAX_COUNT * sizeof(hal_bit_t *));
    gpio_in_out_not = hal_malloc(GPIO_BCM_MAX_COUNT * sizeof(hal_bit_t *));
    gpio_pull = hal_malloc(GPIO_BCM_MAX_COUNT * sizeof(hal_s32_t *));

    if (!gpio_in_out || !gpio_in_out_not || !gpio_pull) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_malloc_init() failed \n");
        return -1;
    }

    return 0;
}

static int32_t gpio_hal_init(void)
{
    int retval;

    char *in_pins_token = strtok(in_pins, ",");
    while (in_pins_token != NULL)
    {
        gpio_in_out_array[gpio_count] = atoi(in_pins_token);
        gpio_in_array[gpio_count] = atoi(in_pins_token);
        gpio_count++;
        in_pins_token = strtok(NULL, ",");
    }

    char *out_pins_token = strtok(out_pins, ",");
    while (out_pins_token != NULL)
    {
        gpio_in_out_array[gpio_count] = atoi(out_pins_token);
        gpio_out_array[gpio_count] = atoi(out_pins_token);
        gpio_count++;
        out_pins_token = strtok(NULL, ",");
    }

    for (int gpio_hal_i = 0; gpio_hal_i < gpio_count; gpio_hal_i++)
    {

        int check = 0;

        if (isInArray(gpio_in_array, sizeof(gpio_in_array) / sizeof(gpio_in_array[0]), gpio_in_out_array[gpio_hal_i]))
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "gpio_in_array \n");

            pinMode(gpio_in_out_array[gpio_hal_i], OUTPUT);

            retval = hal_pin_bit_newf(HAL_OUT, &gpio_in_out[gpio_in_out_array[gpio_hal_i]], component_id, "%s.gpio.pin%d-%s", component_name, gpio_in_out_array[gpio_hal_i], "in");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: gpio_hal_init() in %d failed \n", gpio_in_out_array[gpio_hal_i]);
                return -1;
            }

            retval = hal_pin_bit_newf(HAL_OUT, &gpio_in_out_not[gpio_in_out_array[gpio_hal_i]], component_id, "%s.gpio.pin%d-%s-not", component_name, gpio_in_out_array[gpio_hal_i], "in");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: gpio_hal_init() in-not %d failed \n", gpio_in_out_array[gpio_hal_i]);
                return -1;
            }

            retval = hal_pin_s32_newf(HAL_IN, &gpio_pull[gpio_in_out_array[gpio_hal_i]], component_id, "%s.gpio.pin%d-%s", component_name, gpio_in_out_array[gpio_hal_i], "pull");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: gpio_hal_init() pull %d failed \n", gpio_in_out_array[gpio_hal_i]);
                return -1;
            }

            check = 1;
        }

        if (isInArray(gpio_out_array, sizeof(gpio_out_array) / sizeof(gpio_out_array[0]), gpio_in_out_array[gpio_hal_i]))
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "gpio_out_array \n");

            pinMode(gpio_in_out_array[gpio_hal_i], INPUT);

            retval = hal_pin_bit_newf(HAL_IN, &gpio_in_out[gpio_in_out_array[gpio_hal_i]], component_id, "%s.gpio.pin%d-%s", component_name, gpio_in_out_array[gpio_hal_i], "out");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: gpio_hal_init() out %d failed \n", gpio_in_out_array[gpio_hal_i]);
                return -1;
            }

            retval = hal_pin_bit_newf(HAL_IN, &gpio_in_out_not[gpio_in_out_array[gpio_hal_i]], component_id, "%s.gpio.pin%d-%s-not", component_name, gpio_in_out_array[gpio_hal_i], "out");
            if (retval < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: gpio_hal_init() out-not %d failed \n", gpio_in_out_array[gpio_hal_i]);
                return -1;
            }

            check = 1;
        }

        if (check)
        {
            pullUpDnControl(gpio_in_out_array[gpio_hal_i], PUD_OFF);

            *gpio_in_out[gpio_in_out_array[gpio_hal_i]] = digitalRead(gpio_in_out_array[gpio_hal_i]) == HIGH ? 1 : 0;
            *gpio_in_out_not[gpio_in_out_array[gpio_hal_i]] = *gpio_in_out[gpio_in_out_array[gpio_hal_i]] ? 0 : 1;
            gpio_in_out_prev[gpio_in_out_array[gpio_hal_i]] = *gpio_in_out[gpio_in_out_array[gpio_hal_i]];
            gpio_in_out_not_prev[gpio_in_out_array[gpio_hal_i]] = *gpio_in_out_not[gpio_in_out_array[gpio_hal_i]];

            switch (armcnc_xj3_get_gpio_pull((char)getAlt(gpio_in_out_array[gpio_hal_i]))) {
                case PUD_UP:      *gpio_pull[gpio_in_out_array[gpio_hal_i]] = 1;
                case PUD_DOWN:    *gpio_pull[gpio_in_out_array[gpio_hal_i]] = -1;
                default:          *gpio_pull[gpio_in_out_array[gpio_hal_i]] = 0;
            }

            gpio_pull_prev[gpio_in_out_array[gpio_hal_i]] = *gpio_pull[gpio_in_out_array[gpio_hal_i]];
        }else{
            continue;
        }
    }

    return 0;
}

static int32_t pwm_hal_init(void)
{
    int retval;

    char *pwm_types_token = strtok(pwm_types, ",");
    while (pwm_types_token != NULL)
    {
        pwm_array[pwm_count] = pwm_types_token == "p" ? 1 : 2;
        pwm_count++;
        pwm_types_token = strtok(NULL, ",");
    }

    pwm_hal = hal_malloc(pwm_count * sizeof(pwm_hal_struct));
    if (!pwm_hal) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: pwm_hal_init() pwm_hal failed \n");
        return -1;
    }

    for (int pwm_i = 0; pwm_i < pwm_count; pwm_i++)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "pwm_hal_init \n");

        retval = hal_pin_bit_newf(HAL_IN, &pwm_hal[pwm_i].enable, component_id, "%s.pwm.%d.%s", component_name, pwm_i, "enable");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: pwm_hal_init() enable failed \n");
            return -1;
        }
        *pwm_hal[pwm_i].enable = 0;

        retval = hal_pin_u32_newf(HAL_IN, &pwm_hal[pwm_i].pwm_port, component_id, "%s.pwm.%d.%s", component_name, pwm_i, "pwm-port");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: pwm_hal_init() pwm_port failed \n");
            return -1;
        }
        *pwm_hal[pwm_i].pwm_port = UINT32_MAX;
        retval = hal_pin_u32_newf(HAL_IN, &pwm_hal[pwm_i].pwm_pin, component_id, "%s.pwm.%d.%s", component_name, pwm_i, "pwm-pin");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: pwm_hal_init() pwm_pin failed \n");
            return -1;
        }
        *pwm_hal[pwm_i].pwm_pin = UINT32_MAX;
        retval = hal_pin_bit_newf(HAL_IN, &pwm_hal[pwm_i].pwm_invert, component_id, "%s.pwm.%d.%s", component_name, pwm_i, "pwm-invert");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() pwm_invert failed \n");
            return -1;
        }
        *pwm_hal[pwm_i].pwm_invert = 0;

        retval = hal_pin_u32_newf(HAL_IN, &pwm_hal[pwm_i].dir_port, component_id, "%s.pwm.%d.%s", component_name, pwm_i, "dir-port");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() dir_port failed \n");
            return -1;
        }
        *pwm_hal[pwm_i].dir_port = UINT32_MAX;
        retval = hal_pin_u32_newf(HAL_IN, &pwm_hal[pwm_i].dir_pin, component_id, "%s.pwm.%d.%s", component_name, pwm_i, "dir-pin");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() dir_pin failed \n");
            return -1;
        }
        *pwm_hal[pwm_i].dir_pin = UINT32_MAX;
        retval = hal_pin_bit_newf(HAL_IN, &pwm_hal[pwm_i].dir_invert, component_id, "%s.pwm.%d.%s", component_name, pwm_i, "dir-invert");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() dir_invert failed \n");
            return -1;
        }
        *pwm_hal[pwm_i].dir_invert = 0;
        retval = hal_pin_u32_newf(HAL_IO, &pwm_hal[pwm_i].dir_hold, component_id, "%s.pwm.%d.%s", component_name, pwm_i, "dir-hold");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() dir_hold failed \n");
            return -1;
        }
        *pwm_hal[pwm_i].dir_hold = 50000;
        retval = hal_pin_u32_newf(HAL_IO, &pwm_hal[pwm_i].dir_setup, component_id, "%s.pwm.%d.%s", component_name, pwm_i, "dir-setup");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() dir_setup failed \n");
            return -1;
        }
        *pwm_hal[pwm_i].dir_setup = 50000;

        retval = hal_pin_float_newf(HAL_IN, &pwm_hal[pwm_i].dc_cmd, component_id, "%s.pwm.%d.%s", component_name, pwm_i, "dc-cmd");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() dc_cmd failed \n");
            return -1;
        }
        *pwm_hal[pwm_i].dc_cmd = 0.0;
        retval = hal_pin_float_newf(HAL_IO, &pwm_hal[pwm_i].dc_min, component_id, "%s.pwm.%d.%s", component_name, pwm_i, "dc-min");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() dc_min failed \n");
            return -1;
        }
        *pwm_hal[pwm_i].dc_min = -1.0;
        retval = hal_pin_float_newf(HAL_IO, &pwm_hal[pwm_i].dc_max, component_id, "%s.pwm.%d.%s", component_name, pwm_i, "dc-max");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() dc_max failed \n");
            return -1;
        }
        *pwm_hal[pwm_i].dc_max = 1.0;
        retval = hal_pin_u32_newf(HAL_IO, &pwm_hal[pwm_i].dc_max_t, component_id, "%s.pwm.%d.%s", component_name, pwm_i, "dc-max-t");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() dc_max_t failed \n");
            return -1;
        }
        *pwm_hal[pwm_i].dc_max_t = 0;
        retval = hal_pin_float_newf(HAL_IO, &pwm_hal[pwm_i].dc_offset, component_id, "%s.pwm.%d.%s", component_name, pwm_i, "dc-offset");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() dc_offset failed \n");
            return -1;
        }
        *pwm_hal[pwm_i].dc_offset = 0.0;
        retval = hal_pin_float_newf(HAL_IO, &pwm_hal[pwm_i].dc_scale, component_id, "%s.pwm.%d.%s", component_name, pwm_i, "dc-scale");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() dc_scale failed \n");
            return -1;
        }
        *pwm_hal[pwm_i].dc_scale = 1.0;

        retval = hal_pin_float_newf(HAL_IO, &pwm_hal[pwm_i].pos_scale, component_id, "%s.pwm.%d.%s", component_name, pwm_i, "pos-scale");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() pos_scale failed \n");
            return -1;
        }
        *pwm_hal[pwm_i].pos_scale = 1.0;
        retval = hal_pin_float_newf(HAL_IN, &pwm_hal[pwm_i].pos_cmd, component_id, "%s.pwm.%d.%s", component_name, pwm_i, "pos-cmd");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() pos_cmd failed \n");
            return -1;
        }
        *pwm_hal[pwm_i].pos_cmd = 0.0;

        retval = hal_pin_float_newf(HAL_IO, &pwm_hal[pwm_i].vel_scale, component_id, "%s.pwm.%d.%s", component_name, pwm_i, "vel-scale");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() vel_scale failed \n");
            return -1;
        }
        *pwm_hal[pwm_i].vel_scale = 1.0;
        retval = hal_pin_float_newf(HAL_IN, &pwm_hal[pwm_i].vel_cmd, component_id, "%s.pwm.%d.%s", component_name, pwm_i, "vel-cmd");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() vel_cmd failed \n");
            return -1;
        }
        *pwm_hal[pwm_i].vel_cmd = 0.0;

        retval = hal_pin_float_newf(HAL_IO, &pwm_hal[pwm_i].freq_cmd, component_id, "%s.pwm.%d.%s", component_name, pwm_i, "freq-cmd");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() freq_cmd failed \n");
            return -1;
        }
        *pwm_hal[pwm_i].freq_cmd = 0.0;
        retval = hal_pin_float_newf(HAL_IO, &pwm_hal[pwm_i].freq_min, component_id, "%s.pwm.%d.%s", component_name, pwm_i, "freq-min");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() freq_min failed \n");
            return -1;
        }
        *pwm_hal[pwm_i].freq_min = 50.0;
        retval = hal_pin_float_newf(HAL_IO, &pwm_hal[pwm_i].freq_max, component_id, "%s.pwm.%d.%s", component_name, pwm_i, "freq-max");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() freq_max failed \n");
            return -1;
        }
        *pwm_hal[pwm_i].freq_max = 500000.0;

        retval = hal_pin_float_newf(HAL_OUT, &pwm_hal[pwm_i].dc_fb, component_id, "%s.pwm.%d.%s", component_name, pwm_i, "dc-fb");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() dc_fb failed \n");
            return -1;
        }
        *pwm_hal[pwm_i].dc_fb = 0.0;
        retval = hal_pin_float_newf(HAL_OUT, &pwm_hal[pwm_i].pos_fb, component_id, "%s.pwm.%d.%s", component_name, pwm_i, "pos-fb");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() pos_fb failed \n");
            return -1;
        }
        *pwm_hal[pwm_i].pos_fb = 0.0;
        retval = hal_pin_float_newf(HAL_OUT, &pwm_hal[pwm_i].freq_fb, component_id, "%s.pwm.%d.%s", component_name, pwm_i, "freq-fb");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() freq_fb failed \n");
            return -1;
        }
        *pwm_hal[pwm_i].freq_fb = 0.0;
        retval = hal_pin_float_newf(HAL_OUT, &pwm_hal[pwm_i].vel_fb, component_id, "%s.pwm.%d.%s", component_name, pwm_i, "vel-fb");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() vel_fb failed \n");
            return -1;
        }
        *pwm_hal[pwm_i].vel_fb = 0.0;
        retval = hal_pin_s32_newf(HAL_OUT, &pwm_hal[pwm_i].counts, component_id, "%s.pwm.%d.%s", component_name, pwm_i, "counts");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() vel_fb failed \n");
            return -1;
        }
        *pwm_hal[pwm_i].counts = 0;
    }
}

static int32_t start_init(void)
{
    int retval;

    char name[HAL_NAME_LEN + 1];

    if (in_pins == NULL || in_pins[0] == '\0')
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() in_pins failed \n");
        return -1;
    }

    if (out_pins == NULL || out_pins[0] == '\0')
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() out_pins failed \n");
        return -1;
    }

    if (pwm_types == NULL || pwm_types[0] == '\0')
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() pwm_types failed \n");
        return -1;
    }

    hal_malloc_init();

    gpio_hal_init();

    pwm_hal_init();

    rtapi_snprintf(name, sizeof(name), "%s.gpio.write", component_name);
    retval = hal_export_funct(name, gpio_write, 0, 0, 0, component_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() gpio_write failed \n");
        return -1;
    }

    rtapi_snprintf(name, sizeof(name), "%s.gpio.read", component_name);
    retval = hal_export_funct(name, gpio_read, 0, 0, 0, component_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() gpio_read failed \n");
        return -1;
    }

    rtapi_snprintf(name, sizeof(name), "%s.pwm.write", component_name);
    retval = hal_export_funct(name, pwm_write, 0, 1, 0, component_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() pwm_write failed \n");
        return -1;
    }

    rtapi_snprintf(name, sizeof(name), "%s.pwm.read", component_name);
    retval = hal_export_funct(name, pwm_read, 0, 1, 0, component_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: drives_init() pwm_read failed \n");
        return -1;
    }

    return 0;
}

static void gpio_read(void *arg, long period)
{
    for (int gpio_hal_i = 0; gpio_hal_i < gpio_count; gpio_hal_i++)
    {
        if (isInArray(gpio_in_array, sizeof(gpio_in_array) / sizeof(gpio_in_array[0]), gpio_in_out_array[gpio_hal_i]))
        {
            if (digitalRead(gpio_in_out_array[gpio_hal_i]) == HIGH)
            {
                *gpio_in_out[gpio_in_out_array[gpio_hal_i]] = 1;
                *gpio_in_out_not[gpio_in_out_array[gpio_hal_i]] = 0;
            }else{
                *gpio_in_out[gpio_in_out_array[gpio_hal_i]] = 0;
                *gpio_in_out_not[gpio_in_out_array[gpio_hal_i]] = 1;
            }
        }

        if (isInArray(gpio_out_array, sizeof(gpio_out_array) / sizeof(gpio_out_array[0]), gpio_in_out_array[gpio_hal_i]))
        {
            if (digitalRead(gpio_in_out_array[gpio_hal_i]) == HIGH)
            {
                *gpio_in_out[gpio_in_out_array[gpio_hal_i]] = 1;
                *gpio_in_out_not[gpio_in_out_array[gpio_hal_i]] = 0;
            }else{
                *gpio_in_out[gpio_in_out_array[gpio_hal_i]] = 0;
                *gpio_in_out_not[gpio_in_out_array[gpio_hal_i]] = 1;
            }
        }
    }
}

static void gpio_write(void *arg, long period)
{
}

static void pwm_read(void *arg, long period)
{
}

static void pwm_write(void *arg, long period)
{
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

    if (start_init()) {
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