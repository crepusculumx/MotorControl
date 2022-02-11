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
#include "MotorController.h"

#define MAX_PACKET_LENGTH 50 // 最大包长
#define MIN_PACKET_LENGTH 16 // 最小包长 2(head) + 1(length) + 1(packet_type) + 10(MotorCmdMsg) + 2(crc)

// 包头标识
#define HD_NUM (uint8_t)0xFA
#define ID_NUM (uint8_t)0x01

#define MOTOR_STATE_MSG_LENGTH 10
#define MOTOR_STATE_MSG_FLOAT_TO_INT 1000
typedef struct MotorStateMsg {
  uint8_t id;
  uint8_t mode;
  uint64_t value;
} MotorStateMsg;

#define MOTOR_CMD_MSG_LENGTH 10
#define MOTOR_CMD_MSG_FLOAT_TO_INT 1000
typedef struct MotorCmdMsg {
  uint8_t id;
  uint8_t mode;
  int64_t value;
} MotorCmdMsg;

typedef enum PacketType {
  PACKET_MOTOR_CMD = 0,
  PACKET_MOTOR_STATE = 1
} PacketType;

#define PACKET_BASE_LENGTH 6

typedef struct Packet {
  uint8_t head[2];// HD_NUM ID_NUM
  uint8_t length;
  uint8_t packet_type;
  //Msg
  uint16_t crc;
} Packet;

typedef struct ComProtocolParser {
  Deque deque;

  MotorCmdMsg motor_cmd;
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

int protocol_dump(int id, MotorState motor_state, uint8_t buffer[], int length, size_t *it);

#endif //ROBOTMOTORCONTROL_USER_SRC_COMPROTOCOL_H_
