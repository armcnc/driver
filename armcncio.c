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
static int in_pins_array[GPIO_MAX_COUNT];
static int in_pins_count = 0;

static char *out_pins = "";
#ifdef RTAPI
RTAPI_MP_STRING(out_pins, "channels control type, comma separated");
#endif
static int out_pins_array[GPIO_MAX_COUNT];
static int out_pins_count = 0;

static int32_t component_id;
static const uint8_t * component_name = "armcncio";

static int32_t malloc_and_export(const char *component_name, int32_t component_id)
{
    int port, retval;

    char name[HAL_NAME_LEN + 1];

    if (in_pins == NULL || in_pins[0] == '\0')
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: malloc_and_export() in_pins failed \n");
        return -1;
    }

    if (out_pins == NULL || out_pins[0] == '\0')
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: malloc_and_export() out_pins failed \n");
        return -1;
    }

    gpio_hal_in = hal_malloc(GPIO_MAX_COUNT * sizeof(hal_bit_t *));
    gpio_hal_in_not = hal_malloc(GPIO_MAX_COUNT * sizeof(hal_bit_t *));
    gpio_hal_out = hal_malloc(GPIO_MAX_COUNT * sizeof(hal_bit_t *));
    gpio_hal_out_not = hal_malloc(GPIO_MAX_COUNT * sizeof(hal_bit_t *));
    gpio_hal_up_down = hal_malloc(GPIO_MAX_COUNT * sizeof(hal_s32_t *));

    if (!gpio_hal_in || !gpio_hal_in_not || !gpio_hal_out || !gpio_hal_out_not || !gpio_hal_up_down) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: malloc_and_export() failed code 1 \n");
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
        pinMode(in_pins_array[in_pins_i], INPUT);
        retval = hal_pin_bit_newf(HAL_IN, &gpio_hal_in[in_pins_array[in_pins_i]], component_id, "%s.gpio.pin%d-%s", component_name, in_pins_array[in_pins_i], "in");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: malloc_and_export() failed  code 2 \n");
            return -1;
        }
        retval = hal_pin_bit_newf(HAL_IN, &gpio_hal_in_not[in_pins_array[in_pins_i]], component_id, "%s.gpio.pin%d-%s-not", component_name, in_pins_array[in_pins_i], "in");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: malloc_and_export() failed  code 3 \n");
            return -1;
        }
    }

    char *out_pins_token = strtok(out_pins, ",");
    while (out_pins_token != NULL)
    {
        out_pins_array[in_pins_count] = atoi(out_pins_token);
        out_pins_count++;
        out_pins_token = strtok(NULL, ",");
    }

    for (int out_pins_i = 0; out_pins_i < out_pins_count; out_pins_i++)
    {
        pinMode(out_pins_array[out_pins_i], OUTPUT);
        retval = hal_pin_bit_newf(HAL_OUT, &gpio_hal_out[out_pins_array[out_pins_i]], component_id, "%s.gpio.pin%d-%s", component_name, out_pins_array[out_pins_i], "out");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: malloc_and_export() failed  code 4 \n");
            return -1;
        }
        retval = hal_pin_bit_newf(HAL_OUT, &gpio_hal_out_not[out_pins_array[out_pins_i]], component_id, "%s.gpio.pin%d-%s-not", component_name, out_pins_array[out_pins_i], "out");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: malloc_and_export() failed  code 5 \n");
            return -1;
        }
    }

    rtapi_snprintf(name, sizeof(name), "%s.gpio.write", component_name);
    retval = hal_export_funct(name, gpio_write, 0, 0, 0, component_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: malloc_and_export() failed  code 6 \n");
        return -1;
    }
    rtapi_snprintf(name, sizeof(name), "%s.gpio.read", component_name);
    retval = hal_export_funct(name, gpio_read, 0, 0, 0, component_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: malloc_and_export() failed  code 7 \n");
        return -1;
    }

    return 0;
}

static void gpio_read(void *arg, long period)
{
    for (int in_pins_i = 0; in_pins_i < in_pins_count; in_pins_i++)
    {
        if (digitalRead(in_pins_array[in_pins_i]) == HIGH)
        {
            *gpio_hal_in[in_pins_array[in_pins_i]] = 1;
            *gpio_hal_in_not[in_pins_array[in_pins_i]] = 0;
        }else{
            *gpio_hal_in[in_pins_array[in_pins_i]] = 0;
            *gpio_hal_in_not[in_pins_array[in_pins_i]] = 1;
        }
    }
}

static void gpio_write(void *arg, long period)
{
    for (int out_pins_i = 0; out_pins_i < out_pins_count; out_pins_i++)
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

    if (malloc_and_export(component_name, component_id)) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: malloc_and_export() failed \n");
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