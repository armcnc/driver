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

static char *ins = "";
#ifdef RTAPI
RTAPI_MP_STRING(ins, "channels control type, comma separated");
#endif

static char *outs = "";
#ifdef RTAPI
RTAPI_MP_STRING(outs, "channels control type, comma separated");
#endif

static int32_t component_id;
static const uint8_t * component_name = "armcncio";

static int32_t hal_start(const char *component_name, int32_t component_id)
{
    int retval;

    char name[HAL_NAME_LEN + 1];

    if (ins == NULL || ins[0] == '\0')
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() ins failed \n");
        return -1;
    }

    if (outs == NULL || outs[0] == '\0')
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() outs failed \n");
        return -1;
    }

    gpio_hal = hal_malloc(GPIO_BCM_MAX_COUNT * sizeof(hal_bit_t *));
    gpio_hal_not = hal_malloc(GPIO_BCM_MAX_COUNT * sizeof(hal_bit_t *));

    if (!gpio_hal || !gpio_hal_not) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() gpio_hal failed \n");
        return -1;
    }

    for (int n = 0; n < GPIO_BCM_MAX_COUNT; n++) gpio_mask[n] = 1UL << n;

    char *in_pins_token = strtok(ins, ",");
    while (in_pins_token != NULL)
    {
        in_pins_array[in_pins_count] = atoi(in_pins_token);
        in_pins_count++;
        in_pins_token = strtok(NULL, ",");
    }

    char *out_pins_token = strtok(outs, ",");
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

    pwm_hal = hal_malloc(GPIO_PWM_MAX_COUNT * sizeof(pwm_hal_struct));
    if (!pwm_hal) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal failed \n");
        return -1;
    }

    retval = 0;
    #define PWM_EXPORT_PIN(CH,IO_TYPE,VAR_TYPE,VAL,NAME,DEFAULT) \
        retval += hal_pin_##VAR_TYPE##_newf(IO_TYPE, &(pwm_hal[CH].VAL), component_id,\
        "%s.pwm.%d." NAME, component_name, CH);\
        *pwm_hal[CH].VAL = DEFAULT;\
        pwm_hal_prev[CH].VAL = DEFAULT;
    
    for (int ch = 0; ch < GPIO_PWM_MAX_COUNT; ch++)
    {

        if(ch == 5)
        {
            softPwmCreate(16, 0, 100);
        }

        PWM_EXPORT_PIN(ch, HAL_IN, bit, enable, "enable", 0);

        PWM_EXPORT_PIN(ch, HAL_IO, float, frequency_command, "frequency-command", 0.0);

        PWM_EXPORT_PIN(ch, HAL_IN, float, duty_cycle_command, "duty-cycle-command", 0.0);
        PWM_EXPORT_PIN(ch, HAL_IO, float, duty_cycle_scale, "duty-cycle-scale", 1.0);

        PWM_EXPORT_PIN(ch, HAL_IN, u32, step_port, "step-port", UINT32_MAX);
        PWM_EXPORT_PIN(ch, HAL_IN, bit, step_pin, "step-pin", 0);
        PWM_EXPORT_PIN(ch, HAL_IN, bit, step_pin_not, "step-pin-not", 0);
        PWM_EXPORT_PIN(ch, HAL_IN, u32, step_direction_port, "step-direction-port", UINT32_MAX);
        PWM_EXPORT_PIN(ch, HAL_IN, bit, step_direction_pin, "step-direction-pin", 0);
        PWM_EXPORT_PIN(ch, HAL_IN, bit, step_direction_pin_not, "step-direction-pin-not", 0);

        PWM_EXPORT_PIN(ch, HAL_IN, u32, spindle_pin, "spindle-pin", UINT32_MAX);
        PWM_EXPORT_PIN(ch, HAL_IN, bit, spindle_pin_not, "spindle-pin-not", 0);
        PWM_EXPORT_PIN(ch, HAL_IN, u32, spindle_forward_pin, "spindle-forward-pin", UINT32_MAX);
        PWM_EXPORT_PIN(ch, HAL_IN, bit, spindle_forward_pin_not, "spindle-forward-pin-not", 0);
        PWM_EXPORT_PIN(ch, HAL_IN, u32, spindle_reverse_pin, "spindle-reverse-pin", UINT32_MAX);
        PWM_EXPORT_PIN(ch, HAL_IN, bit, spindle_reverse_pin_not, "spindle-reverse-pin-not", 0);

        pwm_hal_prev[ch].is_init = 0;

        pwm_hal_count++;
    }

    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() PWM_EXPORT_PIN failed \n");
        return -1;
    }

    #undef PWM_EXPORT_PIN

    rtapi_snprintf(name, sizeof(name), "%s.write", component_name);
    retval = hal_export_funct(name, write_port, 0, 0, 0, component_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() gpio_write failed \n");
        return -1;
    }

    rtapi_snprintf(name, sizeof(name), "%s.read", component_name);
    retval = hal_export_funct(name, read_port, 0, 0, 0, component_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() gpio_read failed \n");
        return -1;
    }

    rtapi_snprintf(name, sizeof(name), "%s.pwm-read", component_name);
    retval = hal_export_funct(name, pwm_read, 0, 0, 0, component_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_read failed \n");
        return -1;
    }

    rtapi_snprintf(name, sizeof(name), "%s.pwm-write", component_name);
    retval = hal_export_funct(name, pwm_write, 0, 0, 0, component_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_write failed \n");
        return -1;
    }

    return 0;
}

static void read_port(void *arg, long period)
{
    if (!in_pins_count || !out_pins_count || !pwm_hal_count) return;

    for (int pin = 0; pin < GPIO_BCM_MAX_COUNT; pin++)
    {
        if (!gpio_in_mask[pin]) continue;

        if (!(gpio_in_mask[pin] & gpio_mask[pin])) continue;

        uint32_t pin_state = digitalRead(pin) == HIGH ? gpio_mask[pin] : 0;

        if (pin_state & gpio_mask[pin])
        {
            *gpio_hal[pin] = 1;
            *gpio_hal_not[pin] = 0;
        } else {
            *gpio_hal[pin] = 0;
            *gpio_hal_not[pin] = 1;
        }
    }
}

static void write_port(void *arg, long period)
{
    static uint32_t mask_0, mask_1;

    if (!in_pins_count || !out_pins_count || !pwm_hal_count) return;

    for (int pin = 0; pin < GPIO_BCM_MAX_COUNT; pin++)
    {
        if (!gpio_in_mask[pin] && !gpio_out_mask[pin]) continue;

        mask_0 = 0;
        mask_1 = 0;

        if (!(gpio_in_mask[pin] & gpio_mask[pin]) && !(gpio_out_mask[pin] & gpio_mask[pin])) continue;

        if (!(gpio_out_mask[pin] & gpio_mask[pin])) continue;

        if (*gpio_hal[pin] != gpio_hal_prev[pin])
        {
            if (*gpio_hal[pin] == HIGH)
            {
                *gpio_hal_not[pin] = 0;
                mask_1 |= gpio_mask[pin];
            } else {
                *gpio_hal_not[pin] = 1;
                mask_0 |= gpio_mask[pin];
            }

            gpio_hal_prev[pin] = *gpio_hal[pin];
            gpio_hal_not_prev[pin] = *gpio_hal_not[pin];
        }

        if (*gpio_hal_not[pin] != gpio_hal_not_prev[pin])
        {
            if (*gpio_hal_not[pin] == HIGH)
            {
                *gpio_hal[pin] = 0;
                mask_0 |= gpio_mask[pin];
            } else {
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
    if (!in_pins_count || !out_pins_count || !pwm_hal_count) return;

    for (int ch = 0; ch < GPIO_PWM_MAX_COUNT; ch++)
    {
        continue;
    }
}

static void pwm_write(void *arg, long period)
{
    if (!in_pins_count || !out_pins_count || !pwm_hal_count) return;

    for (int ch = 0; ch < GPIO_PWM_MAX_COUNT; ch++)
    {
        if (ch == 5)
        {
            int spindle_init = spindle_control(ch);
            if (spindle_init)
            {
                continue;
            }
        } else {
            int setp_init = step_control(ch);
            if (setp_init)
            {
                continue;
            }
        }
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