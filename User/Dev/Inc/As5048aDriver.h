//
// Created by crepusculumx on 2022/2/21.
//

#ifndef MOTORCONTROL_USER_DEV_SRC_AS5048ADRIVER_H_
#define MOTORCONTROL_USER_DEV_SRC_AS5048ADRIVER_H_

#endif //MOTORCONTROL_USER_DEV_SRC_AS5048ADRIVER_H_

#include "main.h"
#include "GpioDriver.h"

typedef struct As5048aDriver {
  SPI_HandleTypeDef *hspi;

  GpioDriver gpio_driver_spi_cs;


} As5048aDriver;

void As5048aDriver_init(As5048aDriver *as5048a_driver, SPI_HandleTypeDef *hspi, GpioDriver gpio_driver_spi_cs);

void As5048aDriver_cs_reset(As5048aDriver *as5048a_driver);

void As5048aDriver_cs_set(As5048aDriver *as5048a_driver);

double As5048aDriver_read_data(As5048aDriver *as5048a_driver);

