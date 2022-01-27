//
// Created by crepusculumx on 2021/12/22.
//

#include "Deque.h"

uint8_t Deque_pop_back(Deque *deque) {
  deque->length--;

  size_t back = deque->end == 0 ? MAX_deque_SIZE : deque->end - 1;
  uint8_t res = deque->queue[back];
  deque->end = back;
  return res;
}

void Deque_push_back(Deque *deque, uint8_t num) {
  deque->length++;

  deque->queue[deque->end] = num;
  deque->end++;
  if (deque->end == MAX_deque_SIZE) deque->end = 0;
}

uint8_t Deque_pop_front(Deque *deque) {
  deque->length--;

  uint8_t res = deque->queue[deque->begin];
  deque->begin++;
  if (deque->begin == MAX_deque_SIZE) deque->begin = 0;
  return res;
}

void Deque_push_front(Deque *deque, uint8_t num) {
  deque->length++;

  deque->begin = deque->begin == 0 ? MAX_deque_SIZE - 1 : deque->begin - 1;
  deque->queue[deque->begin] = num;
}