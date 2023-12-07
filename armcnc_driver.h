/**
 ******************************************************************************
 * @file    armcnc_driver.h
 * @author  ARMCNC site:www.armcnc.net github:armcnc.github.io
 ******************************************************************************
 */

#include <stdio.h>
#include <ctype.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "./lib/wiringPi/piHiPri_c.h"
#include "./lib/wiringPi/wiringPi.h"
#include "./lib/wiringPi/wiringPi_c.h"
#include "./lib/wiringPi/softPwm.h"
#include "./lib/wiringPi/softPwm_c.h"

#include "rtapi.h"
#include "rtapi_app.h"
#include "rtapi_math.h"
#include "hal.h"