//
// Created by crepusculumx on 2021/12/19.
//

#include <stdio.h>
#include "MotorDriver.h"

void MotorDriver_init(MotorDriver *motor_driver,
                      PwmDriver *PUL_pwm_driver,
                      GpioDriver *en_gpio_driver,
                      GpioDriver *dir_gpio_driver) {
  motor_driver->PUL_pwm_driver = PUL_pwm_driver;
  motor_driver->en_gpio_driver = en_gpio_driver;
  motor_driver->dir_gpio_driver = dir_gpio_driver;

  // 使能 该驱动器reset是开启
//  GpioDriver_set_pin_set(motor_driver->en_gpio_driver);
////  HAL_Delay(1000);
  GpioDriver_set_pin_reset(motor_driver->en_gpio_driver);
}

void MotorDriver_set_status(MotorDriver *motor_driver, double speed, DIR dir) {
  motor_driver->speed = speed;
  motor_driver->dir = dir;

  switch (dir) {
    case CW:motor_driver->dir_gpio_driver->set_pin_set(motor_driver->dir_gpio_driver);
      break;
    case CCW:motor_driver->dir_gpio_driver->set_pin_reset(motor_driver->dir_gpio_driver);
      break;
    default:break;
  }
  PwmDriver_set_pwm(motor_driver->PUL_pwm_driver, (int) (speed * 800 * 10), 0.5);
}