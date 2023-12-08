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
loadrt armcncio in_pins="2,6,13,26,20,21,25,8,7" out_pins="1,18"
...
```

## 🌞 Development Team

> https://www.armcnc.net