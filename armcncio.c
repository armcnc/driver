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
    gpio_hal_pull = hal_malloc(GPIO_BCM_MAX_COUNT * sizeof(hal_s32_t *));
    gpio_hal_drive = hal_malloc(GPIO_BCM_MAX_COUNT * sizeof(hal_u32_t *));

    if (!gpio_hal || !gpio_hal_not || !gpio_hal_pull || !gpio_hal_drive) {
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

        retval = hal_pin_s32_newf(HAL_IN, &gpio_hal_pull[in_pins_array[in_pins_i]], component_id, "%s.gpio.pin%d-pull", component_name, in_pins_array[in_pins_i]);
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() gpio_hal_pull failed \n");
            return -1;
        }

        retval = hal_pin_u32_newf(HAL_IN, &gpio_hal_drive[in_pins_array[in_pins_i]], component_id, "%s.gpio.pin%d-drive", component_name, in_pins_array[in_pins_i]);
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() gpio_hal_drive failed \n");
            return -1;
        }

        pullUpDnControl(in_pins_array[in_pins_i], PUD_OFF);

        *gpio_hal[in_pins_array[in_pins_i]] = digitalRead(in_pins_array[in_pins_i]) == HIGH ? 1 : 0;
        *gpio_hal_not[in_pins_array[in_pins_i]] = *gpio_hal[in_pins_array[in_pins_i]] ? 0 : 1;
        gpio_hal_prev[in_pins_array[in_pins_i]] = *gpio_hal[in_pins_array[in_pins_i]];
        gpio_hal_not_prev[in_pins_array[in_pins_i]] = *gpio_hal_not[in_pins_array[in_pins_i]];

        switch (armcnc_xj3_get_gpio_pull((char)getAlt(in_pins_array[in_pins_i]))) {
            case PULL_UP:      *gpio_hal_pull[in_pins_array[in_pins_i]] = 1;
            case PULL_DOWN:    *gpio_hal_pull[in_pins_array[in_pins_i]] = -1;
            default:           *gpio_hal_pull[in_pins_array[in_pins_i]] = 0;
        }
        gpio_hal_pull_prev[in_pins_array[in_pins_i]] = *gpio_hal_pull[in_pins_array[in_pins_i]];

        *gpio_hal_drive[in_pins_array[in_pins_i]] = (hal_u32_t)armcnc_xj3_get_pin_drive((char)getAlt(in_pins_array[in_pins_i]));
        gpio_hal_drive_prev[in_pins_array[in_pins_i]] = *gpio_hal_drive[in_pins_array[in_pins_i]];
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

        retval = hal_pin_s32_newf(HAL_IN, &gpio_hal_pull[out_pins_array[out_pins_i]], component_id, "%s.gpio.pin%d-pull", component_name, out_pins_array[out_pins_i]);
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() gpio_hal_pull failed \n");
            return -1;
        }

        retval = hal_pin_u32_newf(HAL_IN, &gpio_hal_drive[out_pins_array[out_pins_i]], component_id, "%s.gpio.pin%d-drive", component_name, out_pins_array[out_pins_i]);
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() gpio_hal_drive failed \n");
            return -1;
        }

        pullUpDnControl(out_pins_array[out_pins_i], PUD_OFF);

        *gpio_hal[out_pins_array[out_pins_i]] = digitalRead(out_pins_array[out_pins_i]) == HIGH ? 1 : 0;
        *gpio_hal_not[out_pins_array[out_pins_i]] = *gpio_hal[out_pins_array[out_pins_i]] ? 0 : 1;
        gpio_hal_prev[out_pins_array[out_pins_i]] = *gpio_hal[out_pins_array[out_pins_i]];
        gpio_hal_not_prev[out_pins_array[out_pins_i]] = *gpio_hal_not[out_pins_array[out_pins_i]];

        switch (armcnc_xj3_get_gpio_pull((char)getAlt(out_pins_array[out_pins_i]))) {
            case PUD_UP:      *gpio_hal_pull[out_pins_array[out_pins_i]] = 1;
            case PUD_DOWN:    *gpio_hal_pull[out_pins_array[out_pins_i]] = -1;
            default:           *gpio_hal_pull[out_pins_array[out_pins_i]] = 0;
        }
        gpio_hal_pull_prev[out_pins_array[out_pins_i]] = *gpio_hal_pull[out_pins_array[out_pins_i]];

        *gpio_hal_drive[out_pins_array[out_pins_i]] = (hal_u32_t)armcnc_xj3_get_pin_drive((char)getAlt(out_pins_array[out_pins_i]));
        gpio_hal_drive_prev[out_pins_array[out_pins_i]] = *gpio_hal_drive[out_pins_array[out_pins_i]];
    }

    char *pwm_hal_token = strtok(pwm_types, ",");
    while (pwm_hal_token != NULL)
    {
        pwm_hal_array[pwm_hal_count] = (strcmp(pwm_hal_token, "p") == 0) ? 1 : 2;
        pwm_hal_count++;
        pwm_hal_token = strtok(NULL, ",");
    }

    if (pwm_hal_count > 0)
    {
        pwm_hal = hal_malloc(pwm_hal_count * sizeof(pwm_hal_struct));
        if (!pwm_hal) {
            rtapi_print_msg(RTAPI_MSG_ERR, "[errot]: hal_start() pwm_hal failed \n");
            return -1;
        }

        retval = 0;
        #define EXPORT_PIN(CH,IO_TYPE,VAR_TYPE,VAL,NAME,DEFAULT) \
            retval += hal_pin_##VAR_TYPE##_newf(IO_TYPE, &(pwm_hal[CH].VAL), component_id,\
            "%s.pwm.%d." NAME, component_name, CH);\
            *pwm_hal[CH].VAL = DEFAULT;\
            pwm_hal_prev[CH].VAL = DEFAULT;

        for (int ch = 0; ch < pwm_hal_count; ch++)
        {
            EXPORT_PIN(ch, HAL_IN, bit, enable, "enable", 0);

            EXPORT_PIN(ch, HAL_IO, float, freq_cmd, "freq-cmd", 0.0);

            EXPORT_PIN(ch, HAL_IN, u32, pwm_pin, "pwm-pin", UINT32_MAX);

            EXPORT_PIN(ch, HAL_IN, u32, forward_pin, "forward-pin", UINT32_MAX);
            EXPORT_PIN(ch, HAL_IN, bit, forward_pin_not, "forward-pin-not", 0);

            EXPORT_PIN(ch, HAL_IN, u32, reverse_pin, "reverse-pin", UINT32_MAX);
            EXPORT_PIN(ch, HAL_IN, bit, reverse_pin_not, "reverse-pin-not", 0);

            EXPORT_PIN(ch, HAL_IN, float, dc_cmd, "dc-cmd", 0.0);
            EXPORT_PIN(ch, HAL_IO, float, dc_scale, "dc-scale", 1.0);

            pwm_hal_prev[ch].ctrl_type = pwm_hal_array[ch];
            pwm_hal_prev[ch].is_init = 0;
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
    if (!in_pins_count || !pwm_hal_count) return;

    for (int pin = 0; pin < GPIO_BCM_MAX_COUNT; pin++)
    {
        if (!gpio_in_mask[pin]) continue;

        if (!(gpio_in_mask[pin] & gpio_mask[pin])) continue;

        int is_pwm_ch = 0;
        for (int ch = 0; ch < pwm_hal_count; ch++)
        {
            if((int)(*pwm_hal[ch].pwm_pin) == pin || (int)(*pwm_hal[ch].forward_pin) == pin || (int)(*pwm_hal[ch].reverse_pin) == pin)
            {
                is_pwm_ch = 1;
            }
        }

        if (is_pwm_ch > 0) continue;

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

static void gpio_write(void *arg, long period)
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

        int is_pwm_ch = 0;
        for (int ch = 0; ch < pwm_hal_count; ch++)
        {
            if((int)(*pwm_hal[ch].pwm_pin) == pin || (int)(*pwm_hal[ch].forward_pin) == pin || (int)(*pwm_hal[ch].reverse_pin) == pin)
            {
                is_pwm_ch = 1;
            }
        }

        if (is_pwm_ch > 0) continue;

        if (gpio_hal_pull_prev[pin] != *gpio_hal_pull[pin])
        {
            if (*gpio_hal_pull[pin] > 0)
            {
                *gpio_hal_pull[pin] = 1;
                pullUpDnControl(pin, PUD_UP);
            } else if (*gpio_hal_pull[pin] < 0){
                *gpio_hal_pull[pin] = -1;
                pullUpDnControl(pin, PUD_DOWN);
            } else {
                *gpio_hal_pull[pin] = 0;
                pullUpDnControl(pin, PUD_OFF);
            }
            gpio_hal_pull_prev[pin] = *gpio_hal_pull[pin];
        }

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
    if (!pwm_hal_count) return;

    for (int ch = 0; ch < pwm_hal_count; ch++)
    {
        if (!(*pwm_hal[ch].enable) || !pwm_hal_prev[ch].is_init) continue;
    }
}

static void pwm_write(void *arg, long period)
{
    if (!pwm_hal_count) return;

    for (int ch = 0; ch < pwm_hal_count; ch++)
    {
        if (pwm_hal_prev[ch].ctrl_type == 1)
        {
            int step_control = pwm_step_control(ch);
            if (step_control) {
                continue;
            }
        }

        if (pwm_hal_prev[ch].ctrl_type == 2)
        {
            int spindle_control = pwm_spindle_control(ch);
            if (spindle_control) {
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