/**
 ******************************************************************************
 * @file    armcnc_driver.c
 * @author  ARMCNC site:www.armcnc.net github:armcnc.github.io
 ******************************************************************************
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <wiringPi.h>

#include "rtapi.h"
#include "rtapi_app.h"
#include "rtapi_math.h"
#include "hal.h"

#define MAX_PINS 40
#define MAX_INI_LINE_LENGTH 255
#define MAX_INI_VALUE_LENGTH 10

static int component_id;
hal_bit_t **port_data;

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

#ifdef RTAPI
MODULE_AUTHOR("ARMCNC");
MODULE_DESCRIPTION("Driver for ARMCNC");
MODULE_LICENSE("GPL");
#endif

int read_ini_file(const char *filename, INI_RESULT *result) {

    rtapi_print_msg(RTAPI_MSG_ERR, "%s\n", filename);

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
            rtapi_print_msg(RTAPI_MSG_ERR, "%s %s\n", key, val);
            if (strcmp(key, "ESTOP_PIN") == 0) {
                char *token;
                int i = 0;
                token = strtok(val, " ");
                rtapi_print_msg(RTAPI_MSG_ERR, "%s\n", token);
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
     int n;
}

static void gpio_read(void *arg, long period)
{
     int n;
     for (n = 0; n < MAX_PINS; n++) {
        *(port_data[n]) = 0;
     }
}

int rtapi_app_main(void)
{
    rtapi_print_msg(RTAPI_MSG_INFO, "armcnc_driver...\n");

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

//    if (wiringPiSetup() == -1){
//        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: wiringPiSetup\n");
//        return -1;
//    }

    component_id = hal_init("armcnc_driver");
    if (component_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: component_id\n");
        return -1;
    }

    port_data = hal_malloc(MAX_PINS * sizeof(hal_bit_t *));
    if (port_data == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: port_data\n");
        hal_exit(component_id);
        return -1;
    }

//    retval = hal_pin_bit_newf(strcmp(ini_data.ESTOP_PIN[2], "IN") == 0 ? HAL_IN : HAL_OUT, &port_data[atoi(ini_data.ESTOP_PIN[1])], component_id, ini_data.ESTOP_PIN[0]);
//    if (retval < 0) {
//        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: ESTOP_PIN\n");
//        hal_exit(component_id);
//        return -1;
//    }
//    //pinMode(atoi(ini_data.ESTOP_PIN[1]), ini_data.ESTOP_PIN[2] == "IN" ? INPUT : OUTPUT);
//
//    retval = hal_pin_bit_newf(strcmp(ini_data.SPINDLE_ENABLE_PIN[2], "IN") == 0 ? HAL_IN : HAL_OUT, &port_data[atoi(ini_data.SPINDLE_ENABLE_PIN[1])], component_id, ini_data.SPINDLE_ENABLE_PIN[0]);
//    if (retval < 0) {
//        rtapi_print_msg(RTAPI_MSG_ERR, "%s\n", env_value);
//        rtapi_print_msg(RTAPI_MSG_ERR, "%s\n", ini_data.ESTOP_PIN[0]);
//        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: SPINDLE_ENABLE_PIN\n");
//        hal_exit(component_id);
//        return -1;
//    }
//    //pinMode(atoi(ini_data.SPINDLE_ENABLE_PIN[1]), ini_data.SPINDLE_ENABLE_PIN[2] == "IN" ? INPUT : OUTPUT);
//
//    retval = hal_pin_bit_newf(strcmp(ini_data.SPINDLE_PWM_PIN[2], "IN") == 0 ? HAL_IN : HAL_OUT, &port_data[atoi(ini_data.SPINDLE_PWM_PIN[1])], component_id, ini_data.SPINDLE_PWM_PIN[0]);
//    if (retval < 0) {
//        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: SPINDLE_PWM_PIN\n");
//        hal_exit(component_id);
//        return -1;
//    }
//    //pinMode(atoi(ini_data.SPINDLE_PWM_PIN[1]), ini_data.SPINDLE_PWM_PIN[2] == "IN" ? INPUT : OUTPUT);
//
//    retval = hal_pin_bit_newf(strcmp(ini_data.X_HOME_PIN[2], "IN") == 0 ? HAL_IN : HAL_OUT, &port_data[atoi(ini_data.X_HOME_PIN[1])], component_id, ini_data.X_HOME_PIN[0]);
//    if (retval < 0) {
//        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: X_HOME_PIN\n");
//        hal_exit(component_id);
//        return -1;
//    }
//    //pinMode(atoi(ini_data.X_HOME_PIN[1]), ini_data.X_HOME_PIN[2] == "IN" ? INPUT : OUTPUT);
//
//    retval = hal_pin_bit_newf(strcmp(ini_data.Y_HOME_PIN[2], "IN") == 0 ? HAL_IN : HAL_OUT, &port_data[atoi(ini_data.Y_HOME_PIN[1])], component_id, ini_data.Y_HOME_PIN[0]);
//    if (retval < 0) {
//        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: Y_HOME_PIN\n");
//        hal_exit(component_id);
//        return -1;
//    }
//    //pinMode(atoi(ini_data.Y_HOME_PIN[1]), ini_data.Y_HOME_PIN[2] == "IN" ? INPUT : OUTPUT);
//
//    retval = hal_pin_bit_newf(strcmp(ini_data.Z_HOME_PIN[2], "IN") == 0 ? HAL_IN : HAL_OUT, &port_data[atoi(ini_data.Z_HOME_PIN[1])], component_id, ini_data.Z_HOME_PIN[0]);
//    if (retval < 0) {
//        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: Z_HOME_PIN\n");
//        hal_exit(component_id);
//        return -1;
//    }
//    //pinMode(atoi(ini_data.Z_HOME_PIN[1]), ini_data.Z_HOME_PIN[2] == "IN" ? INPUT : OUTPUT);
//
//    retval = hal_pin_bit_newf(strcmp(ini_data.A_HOME_PIN[2], "IN") == 0 ? HAL_IN : HAL_OUT, &port_data[atoi(ini_data.A_HOME_PIN[1])], component_id, ini_data.A_HOME_PIN[0]);
//    if (retval < 0) {
//        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: A_HOME_PIN\n");
//        hal_exit(component_id);
//        return -1;
//    }
//    //pinMode(atoi(ini_data.A_HOME_PIN[1]), ini_data.A_HOME_PIN[2] == "IN" ? INPUT : OUTPUT);
//
//    retval = hal_pin_bit_newf(strcmp(ini_data.B_HOME_PIN[2], "IN") == 0 ? HAL_IN : HAL_OUT, &port_data[atoi(ini_data.B_HOME_PIN[1])], component_id, ini_data.B_HOME_PIN[0]);
//    if (retval < 0) {
//        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: B_HOME_PIN\n");
//        hal_exit(component_id);
//        return -1;
//    }
//    //pinMode(atoi(ini_data.B_HOME_PIN[1]), ini_data.B_HOME_PIN[2] == "IN" ? INPUT : OUTPUT);
//
//    retval = hal_pin_bit_newf(strcmp(ini_data.C_HOME_PIN[2], "IN") == 0 ? HAL_IN : HAL_OUT, &port_data[atoi(ini_data.C_HOME_PIN[1])], component_id, ini_data.C_HOME_PIN[0]);
//    if (retval < 0) {
//        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: C_HOME_PIN\n");
//        hal_exit(component_id);
//        return -1;
//    }
//    //pinMode(atoi(ini_data.C_HOME_PIN[1]), ini_data.C_HOME_PIN[2] == "IN" ? INPUT : OUTPUT);

    retval = hal_export_funct("gpio.write", gpio_write, 0, 0, 0, component_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: gpio.write\n");
        hal_exit(component_id);
        return -1;
    }

    retval = hal_export_funct("gpio.read", gpio_read, 0, 0, 0, component_id);
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