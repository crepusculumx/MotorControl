//
// Created by crepusculumx on 2021/12/21.
//

#ifndef ROBOTMOTORCONTROL_USER_SRC_TYPECHANGER_H_
#define ROBOTMOTORCONTROL_USER_SRC_TYPECHANGER_H_

#include <stdint.h>
#include <stddef.h>

int16_t data_int8_to_int16(const uint8_t data[], size_t *it);
int32_t data_int8_to_int32(const uint8_t data[], size_t *it);
int64_t data_int8_to_int64(const uint8_t data[], size_t *it);

void data_int16_to_int8(int16_t num, uint8_t data[], size_t *it);
void data_int32_to_int8(int32_t num, uint8_t data[], size_t *it);
void data_int64_to_int8(int64_t num, uint8_t data[], size_t *it);

#endif //ROBOTMOTORCONTROL_USER_SRC_TYPECHANGER_H_
