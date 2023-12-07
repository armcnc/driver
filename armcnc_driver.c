/**
 ******************************************************************************
 * @file    armcnc_driver.c
 * @author  ARMCNC site:www.armcnc.net github:armcnc.github.io
 ******************************************************************************
 */

#include "armcnc_driver.h"
#include "./lib/wiringPi/piHiPri_c.h"
#include "./lib/wiringPi/wiringPi.h"
#include "./lib/wiringPi/wiringPi_c.h"
#include "./lib/wiringPi/softPwm.h"
#include "./lib/wiringPi/softPwm_c.h"

#include "rtapi.h"
#include "rtapi_app.h"
#include "rtapi_math.h"
#include "hal.h"

#define MAX_INI_LINE_LENGTH 512
#define MAX_INI_VALUE_LENGTH 512
#define RTAPI_BIT(nr) (1UL << (nr))

static int component_id;
static int pins = 40;
hal_bit_t **port_data;
hal_float_t **port_data_float;

typedef struct {
    char ESTOP_PIN[MAX_INI_VALUE_LENGTH][MAX_INI_LINE_LENGTH];
    char SPINDLE_ENABLE_PIN[MAX_INI_VALUE_LENGTH][MAX_INI_LINE_LENGTH];
    char SPINDLE_PWM_PIN[MAX_INI_VALUE_LENGTH][MAX_INI_LINE_LENGTH];
    char X_HOME_PIN[MAX_INI_VALUE_LENGTH][MAX_INI_LINE_LENGTH];
    char Y_HOME_PIN[MAX_INI_VALUE_LENGTH][MAX_INI_LINE_LENGTH];
    char Z_HOME_PIN[MAX_INI_VALUE_LENGTH][MAX_INI_LINE_LENGTH];
    char A_HOME_PIN[MAX_INI_VALUE_LENGTH][MAX_INI_LINE_LENGTH];
    char B_HOME_PIN[MAX_INI_VALUE_LENGTH][MAX_INI_LINE_LENGTH];
    char C_HOME_PIN[MAX_INI_VALUE_LENGTH][MAX_INI_LINE_LENGTH];
} INI_RESULT;

static INI_RESULT ini_data = {0};

MODULE_AUTHOR("ARMCNC");
MODULE_DESCRIPTION("Driver for ARMCNC");
MODULE_LICENSE("GPL");

static void gpio_write(void *arg, long period);
static void gpio_read(void *arg, long period);

void read_ini_trim(char *str)
{
    char *end;
    while (isspace((unsigned char)*str)) str++;
    if (*str == 0)
        return;
    end = str + strlen(str) - 1;
    while (end > str && isspace((unsigned char)*end)) end--;
    *(end + 1) = 0;
}

int read_ini_file(const char *filename, INI_RESULT *result)
{
    FILE *file = fopen(filename, "r");
    if (!file) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: INI_RESULT\n");
        return -1;
    }

    char line[MAX_INI_LINE_LENGTH];
    char key[MAX_INI_LINE_LENGTH];
    char val[MAX_INI_LINE_LENGTH];

    while (fgets(line, MAX_INI_LINE_LENGTH, file) != NULL) {
        if (line[0] == '#' || line[0] == ';' || line[0] == '\n') continue;
        if (sscanf(line, "%[^=] = %[^\n]", key, val) == 2) {
            read_ini_trim(key);
            read_ini_trim(val);
            if (strcmp(key, "ESTOP_PIN") == 0) {
                char *token;
                int i = 0;
                token = strtok(val, " ");
                while (token != NULL && i < MAX_INI_VALUE_LENGTH) {
                    strncpy(result->ESTOP_PIN[i], token, MAX_INI_LINE_LENGTH);
                    result->ESTOP_PIN[i][MAX_INI_LINE_LENGTH - 1] = '\0';
                    token = strtok(NULL, " ");
                    i++;
                }
                continue;
            }
            if (strcmp(key, "SPINDLE_ENABLE_PIN") == 0 && strcmp(val, "") != 0) {
                char *token;
                int i = 0;
                token = strtok(val, " ");
                while (token != NULL && i < MAX_INI_VALUE_LENGTH) {
                    strncpy(result->SPINDLE_ENABLE_PIN[i], token, MAX_INI_LINE_LENGTH);
                    result->SPINDLE_ENABLE_PIN[i][MAX_INI_LINE_LENGTH - 1] = '\0';
                    token = strtok(NULL, " ");
                    i++;
                }
                continue;
            }
            if (strcmp(key, "SPINDLE_PWM_PIN") == 0 && strcmp(val, "") != 0) {
                char *token;
                int i = 0;
                token = strtok(val, " ");
                while (token != NULL && i < MAX_INI_VALUE_LENGTH) {
                    strncpy(result->SPINDLE_PWM_PIN[i], token, MAX_INI_LINE_LENGTH);
                    result->SPINDLE_PWM_PIN[i][MAX_INI_LINE_LENGTH - 1] = '\0';
                    token = strtok(NULL, " ");
                    i++;
                }
                continue;
            }
            if (strcmp(key, "X_HOME_PIN") == 0 && strcmp(val, "") != 0) {
                char *token;
                int i = 0;
                token = strtok(val, " ");
                while (token != NULL && i < MAX_INI_VALUE_LENGTH) {
                    strncpy(result->X_HOME_PIN[i], token, MAX_INI_LINE_LENGTH);
                    result->X_HOME_PIN[i][MAX_INI_LINE_LENGTH - 1] = '\0';
                    token = strtok(NULL, " ");
                    i++;
                }
                continue;
            }
            if (strcmp(key, "Y_HOME_PIN") == 0 && strcmp(val, "") != 0) {
                char *token;
                int i = 0;
                token = strtok(val, " ");
                while (token != NULL && i < MAX_INI_VALUE_LENGTH) {
                    strncpy(result->Y_HOME_PIN[i], token, MAX_INI_LINE_LENGTH);
                    result->Y_HOME_PIN[i][MAX_INI_LINE_LENGTH - 1] = '\0';
                    token = strtok(NULL, " ");
                    i++;
                }
                continue;
            }
            if (strcmp(key, "Z_HOME_PIN") == 0 && strcmp(val, "") != 0) {
                char *token;
                int i = 0;
                token = strtok(val, " ");
                while (token != NULL && i < MAX_INI_VALUE_LENGTH) {
                    strncpy(result->Z_HOME_PIN[i], token, MAX_INI_LINE_LENGTH);
                    result->Z_HOME_PIN[i][MAX_INI_LINE_LENGTH - 1] = '\0';
                    token = strtok(NULL, " ");
                    i++;
                }
                continue;
            }
            if (strcmp(key, "A_HOME_PIN") == 0 && strcmp(val, "") != 0) {
                char *token;
                int i = 0;
                token = strtok(val, " ");
                while (token != NULL && i < MAX_INI_VALUE_LENGTH) {
                    strncpy(result->A_HOME_PIN[i], token, MAX_INI_LINE_LENGTH);
                    result->A_HOME_PIN[i][MAX_INI_LINE_LENGTH - 1] = '\0';
                    token = strtok(NULL, " ");
                    i++;
                }
                continue;
            }
            if (strcmp(key, "B_HOME_PIN") == 0 && strcmp(val, "") != 0) {
                char *token;
                int i = 0;
                token = strtok(val, " ");
                while (token != NULL && i < MAX_INI_VALUE_LENGTH) {
                    strncpy(result->B_HOME_PIN[i], token, MAX_INI_LINE_LENGTH);
                    result->B_HOME_PIN[i][MAX_INI_LINE_LENGTH - 1] = '\0';
                    token = strtok(NULL, " ");
                    i++;
                }
                continue;
            }
            if (strcmp(key, "C_HOME_PIN") == 0 && strcmp(val, "") != 0) {
                char *token;
                int i = 0;
                token = strtok(val, " ");
                while (token != NULL && i < MAX_INI_VALUE_LENGTH) {
                    strncpy(result->C_HOME_PIN[i], token, MAX_INI_LINE_LENGTH);
                    result->C_HOME_PIN[i][MAX_INI_LINE_LENGTH - 1] = '\0';
                    token = strtok(NULL, " ");
                    i++;
                }
                continue;
            }
        }
    }

    fclose(file);
    return 0;
}

static void gpio_write(void *arg, long period)
{
     for (int n = 0; n < pins; n++) {
        if(n == atoi(ini_data.SPINDLE_PWM_PIN[1])){

        }else{
            if (*port_data[n]){
                digitalWrite(n, HIGH);
            }else{
                digitalWrite(n, LOW);
            }
        }
     }
}

static void gpio_read(void *arg, long period)
{
     for (int n = 0; n < pins; n++) {
        if(n == atoi(ini_data.SPINDLE_PWM_PIN[1])){
            *port_data_float[n] = 0.25;
        }else{
            if (*port_data[n]){
                if (digitalRead(n) == HIGH){
                    *port_data[n] = 1;
                }else{
                    *port_data[n] = 0;
                }
            }
        }
     }
}

int rtapi_app_main(void)
{
    char name[HAL_NAME_LEN + 1];

    const char* env_var = "MACHINE_PATH";
    char* env_value = getenv(env_var);
    int retval = 0;

    if (!env_value || strcmp(env_value, "") == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: MACHINE_PATH\n");
        return -1;
    }

    char filePath[1024];
    snprintf(filePath, sizeof(filePath), "/opt/armcnc/configs/%s/machine.user", env_value);
    if(read_ini_file(filePath, &ini_data) != 0){
        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: read_ini_file\n");
        return -1;
    }

    if (wiringPiSetup() == -1){
        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: wiringPiSetup\n");
        return -1;
    }

    component_id = hal_init("armcnc_driver");
    if (component_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: component_id\n");
        return -1;
    }

    port_data = hal_malloc(pins * sizeof(hal_bit_t *));
    if (port_data == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: port_data\n");
        hal_exit(component_id);
        return -1;
    }

    port_data_float = hal_malloc(pins * sizeof(hal_float_t *));
    if (port_data_float == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: port_data\n");
        hal_exit(component_id);
        return -1;
    }

    for (int n = 0; n < pins; n++) {
        if(n == atoi(ini_data.ESTOP_PIN[1])){
            pinMode(atoi(ini_data.ESTOP_PIN[1]), strcmp(ini_data.ESTOP_PIN[2], "IN") == 0 ? INPUT : OUTPUT);
            retval = hal_pin_bit_newf(strcmp(ini_data.ESTOP_PIN[2], "IN") == 0 ? HAL_IN : HAL_OUT, &port_data[n], component_id, ini_data.ESTOP_PIN[0], n);
            if (retval < 0) {
                break;
            }
        }
        if(n == atoi(ini_data.SPINDLE_ENABLE_PIN[1])){
            pinMode(atoi(ini_data.SPINDLE_ENABLE_PIN[1]), strcmp(ini_data.SPINDLE_ENABLE_PIN[2], "IN") == 0 ? INPUT : OUTPUT);
            retval = hal_pin_bit_newf(strcmp(ini_data.SPINDLE_ENABLE_PIN[2], "IN") == 0 ? HAL_IN : HAL_OUT, &port_data[n], component_id, ini_data.SPINDLE_ENABLE_PIN[0], n);
            if (retval < 0) {
                break;
            }
        }
        if(n == atoi(ini_data.SPINDLE_PWM_PIN[1])){
            pinMode(atoi(ini_data.SPINDLE_PWM_PIN[1]), strcmp(ini_data.SPINDLE_PWM_PIN[2], "IN") == 0 ? INPUT : OUTPUT);
            retval = hal_pin_float_newf(strcmp(ini_data.SPINDLE_PWM_PIN[2], "IN") == 0 ? HAL_IN : HAL_OUT, &port_data_float[n], component_id, ini_data.SPINDLE_PWM_PIN[0], n);
            if (retval < 0) {
                break;
            }
        }
        if(n == atoi(ini_data.X_HOME_PIN[1])){
            pinMode(atoi(ini_data.X_HOME_PIN[1]), strcmp(ini_data.X_HOME_PIN[2], "IN") == 0 ? INPUT : OUTPUT);
            retval = hal_pin_bit_newf(strcmp(ini_data.X_HOME_PIN[2], "IN") == 0 ? HAL_IN : HAL_OUT, &port_data[n], component_id, ini_data.X_HOME_PIN[0], n);
            if (retval < 0) {
                break;
            }
        }
        if(n == atoi(ini_data.Y_HOME_PIN[1])){
            pinMode(atoi(ini_data.Y_HOME_PIN[1]), strcmp(ini_data.Y_HOME_PIN[2], "IN") == 0 ? INPUT : OUTPUT);
            retval = hal_pin_bit_newf(strcmp(ini_data.Y_HOME_PIN[2], "IN") == 0 ? HAL_IN : HAL_OUT, &port_data[n], component_id, ini_data.Y_HOME_PIN[0], n);
            if (retval < 0) {
                break;
            }
        }
        if(n == atoi(ini_data.Z_HOME_PIN[1])){
            pinMode(atoi(ini_data.Z_HOME_PIN[1]), strcmp(ini_data.Z_HOME_PIN[2], "IN") == 0 ? INPUT : OUTPUT);
            retval = hal_pin_bit_newf(strcmp(ini_data.Z_HOME_PIN[2], "IN") == 0 ? HAL_IN : HAL_OUT, &port_data[n], component_id, ini_data.Z_HOME_PIN[0], n);
            if (retval < 0) {
                break;
            }
        }
        if(n == atoi(ini_data.A_HOME_PIN[1])){
            pinMode(atoi(ini_data.A_HOME_PIN[1]), strcmp(ini_data.A_HOME_PIN[2], "IN") == 0 ? INPUT : OUTPUT);
            retval = hal_pin_bit_newf(strcmp(ini_data.A_HOME_PIN[2], "IN") == 0 ? HAL_IN : HAL_OUT, &port_data[n], component_id, ini_data.A_HOME_PIN[0], n);
            if (retval < 0) {
                break;
            }
        }
        if(n == atoi(ini_data.B_HOME_PIN[1])){
            pinMode(atoi(ini_data.B_HOME_PIN[1]), strcmp(ini_data.B_HOME_PIN[2], "IN") == 0 ? INPUT : OUTPUT);
            retval = hal_pin_bit_newf(strcmp(ini_data.B_HOME_PIN[2], "IN") == 0 ? HAL_IN : HAL_OUT, &port_data[n], component_id, ini_data.B_HOME_PIN[0], n);
            if (retval < 0) {
                break;
            }
        }
        if(n == atoi(ini_data.C_HOME_PIN[1])){
            pinMode(atoi(ini_data.C_HOME_PIN[1]), strcmp(ini_data.C_HOME_PIN[2], "IN") == 0 ? INPUT : OUTPUT);
            retval = hal_pin_bit_newf(strcmp(ini_data.C_HOME_PIN[2], "IN") == 0 ? HAL_IN : HAL_OUT, &port_data[n], component_id, ini_data.C_HOME_PIN[0], n);
            if (retval < 0) {
                break;
            }
        }
    }

    retval = hal_export_funct("armcnc_driver.gpio.write", gpio_write, 0, 0, 0, component_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: gpio.write\n");
        hal_exit(component_id);
        return -1;
    }

    retval = hal_export_funct("armcnc_driver.gpio.read", gpio_read, 0, 0, 0, component_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: gpio.read\n");
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