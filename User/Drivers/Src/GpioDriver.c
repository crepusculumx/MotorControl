//
// Created by crepusculumx on 2021/12/19.
//

#include "GpioDriver.h"

void GpioDriver_init(GpioDriver *gpio_driver, GPIO_TypeDef *GPIOx, uint16_t GPIO_pin) {
  gpio_driver->GPIOx = GPIOx;
  gpio_driver->GPIO_pin = GPIO_pin;

  //func
  gpio_driver->set_pin_set = GpioDriver_set_pin_set;
  gpio_driver->set_pin_reset = GpioDriver_set_pin_reset;
}

void GpioDriver_set_pin_set(GpioDriver *gpio_driver) {
  gpio_driver->pin_state = GPIO_PIN_SET;
  HAL_GPIO_WritePin(gpio_driver->GPIOx, gpio_driver->GPIO_pin, GPIO_PIN_SET);
}

void GpioDriver_set_pin_reset(GpioDriver *gpio_driver) {
  gpio_driver->pin_state = GPIO_PIN_RESET;
  HAL_GPIO_WritePin(gpio_driver->GPIOx, gpio_driver->GPIO_pin, GPIO_PIN_RESET);
}