//
// Created by crepusculumx on 2021/12/19.
//

#ifndef ROBOTMOTORCONTROL_USER_INC_MOTORDRIVER_H_
#define ROBOTMOTORCONTROL_USER_INC_MOTORDRIVER_H_

#include "main.h"
#include "PwmDriver.h"
#include "GpioDriver.h"

typedef enum DIR {
  CW,
  CCW
} DIR;


typedef struct MotorDriver {
  PwmDriver *PUL_pwm_driver;
  GpioDriver *en_gpio_driver;
  GpioDriver *dir_gpio_driver;

  double speed; // 转速 转每秒
  DIR dir; // 方向
} MotorDriver;

void MotorDriver_init(MotorDriver *motor_driver,
                      PwmDriver *PUL_pwm_driver,
                      GpioDriver *en_gpio_driver,
                      GpioDriver *dir_gpio_driver);

void MotorDriver_set_status(MotorDriver *motor_driver, double speed, DIR dir);

#endif //ROBOTMOTORCONTROL_USER_INC_MOTORDRIVER_H_
