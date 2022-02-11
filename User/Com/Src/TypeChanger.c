//
// Created by crepusculumx on 2021/12/21.
//

#include <stdio.h>
#include "TypeChanger.h"

int16_t data_int8_to_int16(const uint8_t data[], size_t *it) {
  uint16_t res = (int16_t) data[*it + 1];
  res = (res << 8) + (int16_t) data[*it];
  *it += 2;
  return (int16_t) res;
}

int32_t data_int8_to_int32(const uint8_t data[], size_t *it) {
  uint32_t res = 0;
  for (int i = 3; i >= 0; i--) {
    res = (res << 8) + data[*it + i];
  }
  *it += 4;
  return (int32_t) res;
}

int64_t data_int8_to_int64(const uint8_t data[], size_t *it) {
  uint64_t res = 0;
  for (int i = 7; i >= 0; i--) {
    res = (res << 8) + data[*it + i];
  }
  *it += 8;
  return (int64_t) res;
}

void data_int16_to_int8(int16_t num, uint8_t data[], size_t *it) {
  uint8_t *t = (uint8_t *) &num;
  for (size_t i = 0; i < 2; i++) {
    data[*it + i] = t[i];
  }
  *it += 2;
}

void data_int32_to_int8(int32_t num, uint8_t data[], size_t *it) {
  uint8_t *t = (uint8_t *) &num;
  for (size_t i = 0; i < 4; i++) {
    data[*it + i] = t[i];
  }
  *it += 4;
}

void data_int64_to_int8(int64_t num, uint8_t data[], size_t *it) {
  uint8_t *t = (uint8_t *) &num;
  for (size_t i = 0; i < 8; i++) {
    data[*it + i] = t[i];
  }
  *it += 8;
}