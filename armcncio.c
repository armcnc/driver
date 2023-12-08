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
RTAPI_MP_STRING(in_pins, "input pins");
#endif
static int in_pins_array[GPIO_MAX_COUNT];
static int in_pins_count = 0;

static char *out_pins = "";
#ifdef RTAPI
RTAPI_MP_STRING(out_pins, "input pins");
#endif
static int out_pins_array[GPIO_MAX_COUNT];
static int out_pins_count = 0;

static int32_t component_id;
static const uint8_t * component_name = "armcncio";

static int32_t malloc_and_export(const char *component_name, int32_t component_id)
{
    int port, retval;

    gpio_hal = hal_malloc(GPIO_MAX_COUNT * sizeof(hal_bit_t *));
    gpio_hal_not = hal_malloc(GPIO_MAX_COUNT * sizeof(hal_bit_t *));
    gpio_hal_up_down = hal_malloc(GPIO_MAX_COUNT * sizeof(hal_s32_t *));

    if (!gpio_hal[port] || !gpio_hal_not[port] || !gpio_hal_up_down[port]) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: malloc_and_export() failed \n");
        return -1;
    }

    char *in_pins_token = strtok(in_pins, ",");
    while (in_pins_token != NULL)
    {
        in_pins_array[in_pins_count] = atoi(in_pins_token);
        in_pins_count++;
        in_pins_token = strtok(NULL, ",");
    }

    for (int in_pins_i = 0; in_pins_i < in_pins_count; in_pins_i++) {
    {
        retval = hal_pin_bit_newf(HAL_IN, &gpio_hal[in_pins_array[in_pins_i]], component_id, "%s.gpio.pin%d-%s", component_name, in_pins_array[in_pins_i], "in");
        retval += hal_pin_bit_newf(HAL_IN, &gpio_hal_not[in_pins_array[in_pins_i]], component_id, "%s.gpio.pin%d-%s-not", component_name, in_pins_array[in_pins_i], "in");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: malloc_and_export() failed \n");
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

    for (int out_pins_i = 0; out_pins_i < out_pins_count; out_pins_i++) {
    {
        retval = hal_pin_bit_newf(HAL_OUT, &gpio_hal[out_pins_array[out_pins_i]], component_id, "%s.gpio.pin%d-%s", component_name, out_pins_array[out_pins_i], "out");
        retval += hal_pin_bit_newf(HAL_OUT, &gpio_hal_not[out_pins_array[out_pins_i]], component_id, "%s.gpio.pin%d-%s-not", component_name, out_pins_array[out_pins_i], "out");
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: malloc_and_export() failed \n");
            return -1;
        }
    }

    return 0;
}

static void gpio_read(void *arg, long period)
{
    for (port = gpio_ports_cnt; port--;)
    {
        
    }
}

static void gpio_write(void *arg, long period)
{
    for (port = gpio_ports_cnt; port--;)
    {
        
    }
}

int rtapi_app_main(void)
{
    if ((component_id = hal_init(component_name)) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: hal_init() failed \n");
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