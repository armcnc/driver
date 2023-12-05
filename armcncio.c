/**
 ******************************************************************************
 * @file    armcncio.go
 * @author  ARMCNC site:www.armcnc.net github:armcnc.github.io
 ******************************************************************************
 */

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <wiringPi.h>

#include "rtapi.h"
#include "rtapi_bitops.h"
#include "rtapi_app.h"
#include "hal.h"

static int comp_id;

int rtapi_app_main(void)
{
    rtapi_print_msg(RTAPI_MSG_INFO, "armcncio...\n");

    const char* env_var = "MACHINE_PATH";

    char* value = getenv(env_var);
    if (!value) {
        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: MACHINE_PATH\n");
        return -1;
    }
    if (value == "") {
        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: MACHINE_PATH\n");
        return -1;
    }

    if (wiringPiSetup() == -1)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "[error]: wiringPiSetup\n");
        return -1;
    }

}

void rtapi_app_exit(void)
{
    hal_exit(comp_id);
}