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
            return 0;
        }
    }

    return -1;
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
        gpio_in_out_array[gpio_in_out_count] = atoi(in_pins_token);
        gpio_in_array[gpio_in_out_count] = atoi(in_pins_token);
        gpio_in_out_count++;
        in_pins_token = strtok(NULL, ",");
    }

    char *out_pins_token = strtok(out_pins, ",");
    while (out_pins_token != NULL)
    {
        gpio_in_out_array[gpio_in_out_count] = atoi(out_pins_token);
        gpio_out_array[gpio_in_out_count] = atoi(out_pins_token);
        gpio_in_out_count++;
        out_pins_token = strtok(NULL, ",");
    }

    for (int gpio_hal_i = 0; gpio_hal_i < gpio_in_out_count; gpio_hal_i++)
    {
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

        retval = hal_pin_s32_newf(HAL_IN, &gpio_pull[gpio_in_out_array[gpio_hal_i]], component_id, "%s.gpio.pin%d-%s", component_name, gpio_in_out_array[gpio_hal_i], "pull");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: gpio_hal_init() pull %d failed \n", gpio_in_out_array[gpio_hal_i]);
            return -1;
        }

        pullUpDnControl(gpio_in_out_array[gpio_hal_i], PUD_OFF);

        if (isInArray(gpio_in_array, sizeof(gpio_in_array) / sizeof(gpio_in_array[0]), gpio_in_out_array[gpio_hal_i]) < 0)
        {
            pinMode(gpio_in_out_array[gpio_hal_i], INPUT);
        }else{
            pinMode(gpio_in_out_array[gpio_hal_i], OUTPUT);
        }

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
    }

    return 0;
}

static int32_t start_init(const char *component_name, int32_t component_id)
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

    if (start_init(component_name, component_id)) {
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