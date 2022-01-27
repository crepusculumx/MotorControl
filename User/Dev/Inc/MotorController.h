//
// Created by crepusculumx on 2021/12/23.
//

#ifndef ROBOTMOTORCONTROL_USER_SRC_MOTORCONTROLLER_H_
#define ROBOTMOTORCONTROL_USER_SRC_MOTORCONTROLLER_H_

#include "main.h"

#include "MotorDriver.h"

//typedef struct MotorState {
//  int mode;
//  double current;
//  double velocity; // 转每秒
//  double position;
//} MotorState;

typedef enum MotorMode {
  VELOCITY,
  POSITION,
  CURRENT
} MotorMode;

typedef struct MotorState {
  MotorMode mode;
  double value;
} MotorState;

typedef struct MotorController {
  MotorDriver *motor_driver;

  MotorState lst_motor_state;
  MotorState cur_motor_state;
  MotorState tar_motor_state;

  double max_acc;//转每秒方

  int control_rate;//Hz
} MotorController;

/**
 * @param motor_controller
 * @param motor_driver
 * @param control_rate 控制频率 Hz
 * @param max_acc 最大加速度 r/s^2
 */
void MotorController_init(MotorController *motor_controller,
                          MotorDriver *motor_driver,
                          int control_rate,
                          double max_acc);

void MotorController_set_target(MotorController *motor_controller, MotorState motor_state);

/**
 * @brief 周期调用，以控制电机
 * @param motor_controller
 */
void MotorController_control(MotorController *motor_controller);

#endif //ROBOTMOTORCONTROL_USER_SRC_MOTORCONTROLLER_H_
