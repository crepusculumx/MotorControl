//
// Created by crepusculumx on 2021/12/20.
//

#include <stdio.h>
#include "ComProtocol.h"

void ComProtocolParser_input(ComProtocolParser *com_protocol_controller, uint8_t rx_data) {
  Deque_push_back(&com_protocol_controller->deque, rx_data);
}

int ComProtocolParser_parse(ComProtocolParser *com_protocol_controller) {
  if (com_protocol_controller->deque.length < MAX_PACKET_LENGTH) return 1;

  uint8_t packet[MAX_PACKET_LENGTH];

  // 先找个包头
  uint8_t temp_lst_head = 0;
  bool find_head_flag = false;
  while (com_protocol_controller->deque.length > 0) {
    uint8_t cur = Deque_pop_front(&com_protocol_controller->deque);
    if (temp_lst_head == HD_NUM && cur == ID_NUM) {
      find_head_flag = true;
      break;
    }
    temp_lst_head = cur;
  }

  // 没有包头
  if (find_head_flag == false) {
    return -1;
  }

  packet[0] = HD_NUM;
  packet[1] = ID_NUM;


  // 包长不够
  if (com_protocol_controller->deque.length == 0) {
    return 1;
  }
  uint8_t length = packet[2] = Deque_pop_front(&com_protocol_controller->deque);

  // 包长错误了
  if (length > MAX_PACKET_LENGTH) {
    return -2;

  }
  // 还不够先把包头和包长放回去
  if (com_protocol_controller->deque.length < length - 3 + 7) {
    Deque_push_front(&com_protocol_controller->deque, length);
    Deque_push_front(&com_protocol_controller->deque, ID_NUM);
    Deque_push_front(&com_protocol_controller->deque, HD_NUM);
    return 2;
  }

  for (size_t i = 3; i < length; i++) {
    packet[i] = Deque_pop_front(&com_protocol_controller->deque);
  }

  // crc校验
  uint16_t crc_res = CRC16_check(packet, length - 2);
  size_t it = length - 2;
  if (crc_res != (uint16_t) data_int8_to_int16(packet, &it)) {
    return -3;
  }

  // 包内容种类
  PacketType packet_type = packet[3];
  switch (packet_type) {
    case PACKET_MOTOR_CMD:
      return 100 + ComProtocolController_parse_motor_cmd(com_protocol_controller,
                                                         &packet[4],
                                                         length - 4 - 2);
      break;
    default:return -4;
      break;
  }
}

int ComProtocolController_parse_motor_cmd(ComProtocolParser *com_protocol_controller,
                                          uint8_t data[],
                                          uint8_t length) {
  if (length != 5) return -1;

  com_protocol_controller->motor_cmd.mode = data[0];
  size_t it = 1;
  com_protocol_controller->motor_cmd.value = data_int8_to_int64(data, &it);

  // 通知获得消息
  com_protocol_controller->motor_cmd_ready_flag = true;
  return 0;
}

bool ComProtocolParser_have_receive(ComProtocolParser *com_protocol_controller) {
  return com_protocol_controller->deque.length > MIN_PACKET_LENGTH;
}