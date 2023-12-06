/**
 ******************************************************************************
 * @file    armcnc_driver.h
 * @author  ARMCNC site:www.armcnc.net github:armcnc.github.io
 ******************************************************************************
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <wiringPi.h>

static void gpio_write(void *arg, long period);
static void gpio_read(void *arg, long period);