//
// Created by crepusculumx on 2021/12/20.
//

#ifndef ROBOTMOTORCONTROL_USER_SRC_COMPROTOCOL_H_
#define ROBOTMOTORCONTROL_USER_SRC_COMPROTOCOL_H_

#include <stdbool.h>
#include <string.h>

#include "TypeChanger.h"
#include "CrcCheck.h"
#include "Deque.h"

#define MAX_PACKET_LENGTH 50 // 最大包长
#define MIN_PACKET_LENGTH 11 // 最小包长

// 包头标识
#define HD_NUM (uint8_t)0xFA
#define ID_NUM (uint8_t)0x01

// 解析目标
typedef struct MotorCmd {
  uint8_t mode;
  int64_t value;
} MotorCmd;

typedef enum PacketType {
  PACKET_MOTOR_CMD = 0
} PacketType;

typedef struct ComProtocolParser {
  Deque deque;

  MotorCmd motor_cmd;
  bool motor_cmd_ready_flag;
} ComProtocolParser;

/**
 * @brief 输入Rx字节
 * @param com_protocol_controller
 * @param rx_data Rx字节
 */
void ComProtocolParser_input(ComProtocolParser *com_protocol_controller, uint8_t rx_data);

/**
 * @brief 周期调用，以解析传输内容
 * @param com_protocol_controller
 */
int ComProtocolParser_parse(ComProtocolParser *com_protocol_controller);

int ComProtocolController_parse_motor_cmd(ComProtocolParser *com_protocol_controller,
                                          uint8_t data[],
                                          uint8_t length);

bool ComProtocolParser_have_receive(ComProtocolParser *com_protocol_controller);

#endif //ROBOTMOTORCONTROL_USER_SRC_COMPROTOCOL_H_
