/**
 ******************************************************************************
 * @file    armcnc_driver.c
 * @author  ARMCNC site:www.armcnc.net github:armcnc.github.io
 ******************************************************************************
 */

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <wiringPi.h>

#include "rtapi.h"
#include "rtapi_bitops.h"
#include "rtapi_app.h"
#include "hal.h"

#define MAX_INI_LINE_LENGTH 255
#define MAX_INI_VALUE_LENGTH 10

static int component_id;
hal_bit_t **port_data;

typedef struct {
    char ESTOP_PIN[MAX_INI_VALUE_LENGTH][MAX_INI_LINE_LENGTH];
} INI_RESULT;

static INI_RESULT ini_data = {0};

#ifdef RTAPI
MODULE_AUTHOR("ARMCNC");
MODULE_DESCRIPTION("Driver for ARMCNC");
MODULE_LICENSE("GPL");
#endif

int read_ini_file(const char *filename, INI_RESULT *result) {

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
        if (sscanf(line, "%[^=] = %s", key, val) == 2) {
            if (strcmp(key, "ESTOP_PIN") == 0 && strcmp(val, "") != 0) {
                char *token;
                int i = 0;
                token = strtok(val, " ");
                while (token != NULL && i < MAX_INI_VALUE_LENGTH) {
                    strncpy(result->ESTOP_PIN[i], token, MAX_INI_LINE_LENGTH);
                    result->ESTOP_PIN[i][MAX_INI_LINE_LENGTH - 1] = '\0';
                    token = strtok(NULL, " ");
                    i++;
                }
                break;
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
    if(!read_ini_file(filePath, &ini_data)){
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