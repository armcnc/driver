# 🛠️ ARMCNC GPIO Driver

⚡ GPIO Driver development framework for armcnc. ⚡

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## 📖 Development



```shell
git clone https://gitee.com/study-dp/WiringPi.git
cd WiringPi
./build
```

```shell
git clone git@github.com:armcnc/driver.git
cd driver
sudo halcompile --install armcnc_driver.c | grep Linking
```

## 📖 Use

```shell
loadrt [KINS]KINEMATICS
loadrt [EMCMOT]EMCMOT base_period_nsec=[EMCMOT]BASE_PERIOD servo_period_nsec=[EMCMOT]SERVO_PERIOD num_joints=[KINS]JOINTS
loadusr -W armcnc_driver
...
```

## 🌞 Development Team

> https://www.armcnc.net