//
// Created by crepusculumx on 2021/12/19.
//

#ifndef ROBOTMOTORCONTROL_USER_INC_GPIODRIVER_H_
#define ROBOTMOTORCONTROL_USER_INC_GPIODRIVER_H_

#include "main.h"

typedef struct GpioDriver {
  // gpio setting

  GPIO_TypeDef *GPIOx;
  uint16_t GPIO_pin;

  // var

  GPIO_PinState pin_state;

  // func

  void (*set_pin_set)(struct GpioDriver *gpio_driver);
  void (*set_pin_reset)(struct GpioDriver *gpio_driver);
} GpioDriver;

void GpioDriver_init(GpioDriver *gpio_driver, GPIO_TypeDef *GPIOx, uint16_t GPIO_pin);

void GpioDriver_set_pin_set(GpioDriver *gpio_driver);

void GpioDriver_set_pin_reset(GpioDriver *gpio_driver);

#endif //ROBOTMOTORCONTROL_USER_INC_GPIODRIVER_H_
