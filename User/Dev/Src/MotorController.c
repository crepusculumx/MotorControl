//
// Created by crepusculumx on 2021/12/23.
//

#include <stdio.h>
#include "MotorController.h"

void MotorController_init(MotorController *motor_controller,
                          MotorDriver *motor_driver,
                          int control_rate,
                          double max_acc) {
  motor_controller->motor_driver = motor_driver;
  motor_controller->control_rate = control_rate;
  motor_controller->max_acc = max_acc;
}

void MotorController_set_cmd(MotorController *motor_controller, MotorCmd motor_cmd) {
  motor_controller->tar_motor_cmd = motor_cmd;
}

void MotorController_set_velocity(MotorController *motor_controller, double velocity) {
  motor_controller->cur_motor_state.velocity = velocity;
}

void MotorController_set_position(MotorController *motor_controller, double position) {
  motor_controller->cur_motor_state.position = position;
}

/**
 * @brief 根据tar_motor_state 和 lst_motor_state，将当前应该控制的状态写入cur_motor_state
 * @param motor_controller
 */
void MotorController_control_velocity(MotorController *motor_controller, double vel) {
  double cur_vel = motor_controller->cur_motor_state.velocity;

  // 限制加速度
  if (vel > cur_vel) { // 加速
    double max_tar_vel = cur_vel + motor_controller->max_acc / motor_controller->control_rate;
    vel = vel > max_tar_vel ? max_tar_vel : vel;
  } else if (vel < cur_vel) { // 减速
    double min_tar_vel = cur_vel - motor_controller->max_acc / motor_controller->control_rate;
    vel = vel < min_tar_vel ? min_tar_vel : vel;
  }

  // 根据cur_motor_state控制pwm
  if (vel > 0) {
    MotorDriver_set_status(motor_controller->motor_driver, vel, CW);
  } else if (vel < 0) {
    MotorDriver_set_status(motor_controller->motor_driver, -vel, CCW);
  } else {
    //速度为0别切换方向
    MotorDriver_set_status(motor_controller->motor_driver, 0, motor_controller->motor_driver->dir);
  }

  motor_controller->cur_motor_state.velocity = vel;
}

void MotorController_control_position(MotorController *motor_controller, double pos) {
  double cur_pos = motor_controller->cur_motor_state.position;

  const double kp = 0.001, ki = 0, kd = 0;
  double cur_err = pos - cur_pos;
  double lst_err = motor_controller->pos_pid_last_err;
  double integral = motor_controller->pos_pid_integral;
  double v = kp * cur_err + ki * integral + kd * (cur_err - lst_err);
  MotorController_control_velocity(motor_controller, v);
}

void MotorController_control(MotorController *motor_controller) {
  // 速度环
  if (motor_controller->tar_motor_cmd.mode == MOTOR_MODE_VELOCITY) {
    MotorController_control_velocity(motor_controller, motor_controller->tar_motor_cmd.value);
  }
  if (motor_controller->tar_motor_cmd.mode == MOTOR_MODE_POSITION) {
    MotorController_control_position(motor_controller, motor_controller->tar_motor_cmd.value);
  }
  motor_controller->lst_motor_state = motor_controller->cur_motor_state;
}