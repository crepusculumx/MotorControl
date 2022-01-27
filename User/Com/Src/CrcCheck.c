//
// Created by crepusculumx on 2021/12/21.
//

#include "CrcCheck.h"

uint16_t CRC16_check(const uint8_t data[], size_t length) {
  uint16_t crc = 0;
  while (length--) {
    crc = (crc >> 8) ^ crc_table[(crc ^ *data++) & 0xff];
  }
  return crc;
}
