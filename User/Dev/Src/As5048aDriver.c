//
// Created by crepusculumx on 2022/2/21.
//

#include "As5048aDriver.h"

void As5048aDriver_init(As5048aDriver *as5048a_driver, SPI_HandleTypeDef *hspi, GpioDriver gpio_driver_spi_cs) {
  as5048a_driver->hspi = hspi;
  as5048a_driver->gpio_driver_spi_cs = gpio_driver_spi_cs;
}

void As5048aDriver_cs_reset(As5048aDriver *as5048a_driver) {
  GpioDriver_set_pin_reset(&as5048a_driver->gpio_driver_spi_cs);
}

void As5048aDriver_cs_set(As5048aDriver *as5048a_driver) {
  GpioDriver_set_pin_set(&as5048a_driver->gpio_driver_spi_cs);
}

double As5048aDriver_read_data(As5048aDriver *as5048a_driver) {
  uint16_t read_angle_value;
  uint16_t cmd = 0xFFFF;
  As5048aDriver_cs_reset(as5048a_driver);
  HAL_SPI_Transmit(as5048a_driver->hspi, (unsigned char *) &cmd, 1, 1000);
  As5048aDriver_cs_set(as5048a_driver);

  As5048aDriver_cs_reset(as5048a_driver);
  HAL_SPI_TransmitReceive(as5048a_driver->hspi,
                          (unsigned char *) &cmd,
                          (unsigned char *) &read_angle_value,
                          1,
                          1000);
  As5048aDriver_cs_set(as5048a_driver);

  return (double) (read_angle_value & 0x3FFF) * 360.0 * 2/ 32767.0;
}