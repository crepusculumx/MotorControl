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
  MOTOR_MODE_VELOCITY = 0,
  MOTOR_MODE_POSITION,
  MOTOR_MODE_CURRENT
} MotorMode;

typedef struct MotorState {
  double velocity;
  double position;
} MotorState;

typedef struct MotorCmd {
  MotorMode mode;
  double value;
} MotorCmd;

typedef struct MotorController {
  MotorDriver *motor_driver;

  MotorState lst_motor_state;

  // 认为cur_motor_state就是当前的状态，motor_driver设置后进瞬间达到
  MotorState cur_motor_state;

  MotorCmd tar_motor_cmd;

  double max_acc;//转每秒方

  int control_rate;//Hz

  double pos_pid_last_err;
  double pos_pid_integral;
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

void MotorController_set_cmd(MotorController *motor_controller, MotorCmd motor_cmd);

void MotorController_set_velocity(MotorController *motor_controller, double velocity);

void MotorController_set_position(MotorController *motor_controller, double position);
/**
 * @brief 周期调用，以控制电机
 * @param motor_controller
 */
void MotorController_control(MotorController *motor_controller);

#endif //ROBOTMOTORCONTROL_USER_SRC_MOTORCONTROLLER_H_
