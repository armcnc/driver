# 🛠️ ARMCNC GPIO Driver

⚡ GPIO Driver development framework for armcnc. ⚡

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## 📖 Development

```shell
git clone git@github.com:armcnc/driver.git
cd driver
sudo halcompile --install armcncio.c | grep Linking
```

## 📖 Use

```shell
loadrt [KINS]KINEMATICS
loadrt [EMCMOT]EMCMOT base_period_nsec=[EMCMOT]BASE_PERIOD servo_period_nsec=[EMCMOT]SERVO_PERIOD num_joints=[KINS]JOINTS
loadrt armcncio pwm_types="p,p,p,p,p,f" in_pins="19,20,21" out_pins="4,5,6"
...
```

## 🌞 Development Team

> https://www.armcnc.net