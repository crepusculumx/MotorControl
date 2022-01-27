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

void MotorController_set_target(MotorController *motor_controller, MotorState motor_state) {
  motor_controller->tar_motor_state = motor_state;
}

/**
 * @brief 根据tar_motor_state 和 lst_motor_state，将当前应该控制的状态写入cur_motor_state
 * @param motor_controller
 */
void MotorController_control_vel(MotorController *motor_controller) {
  double vel = motor_controller->tar_motor_state.value;

  // 加速
  if (vel > motor_controller->lst_motor_state.value) {
    // 限制加速度
    double max_tar_vel =
        motor_controller->lst_motor_state.value + motor_controller->max_acc / motor_controller->control_rate;
    motor_controller->cur_motor_state.value = vel > max_tar_vel ? max_tar_vel : vel;
  } else if (vel < motor_controller->lst_motor_state.value) {
    double min_tar_vel =
        motor_controller->lst_motor_state.value - motor_controller->max_acc / motor_controller->control_rate;
    motor_controller->cur_motor_state.value = vel < min_tar_vel ? min_tar_vel : vel;
  }

  // 根据cur_motor_state控制pwm
  if (motor_controller->cur_motor_state.value > 0) {
    MotorDriver_set_status(motor_controller->motor_driver, motor_controller->cur_motor_state.value, CW);
  } else if (motor_controller->cur_motor_state.value < 0) {
    MotorDriver_set_status(motor_controller->motor_driver, -motor_controller->cur_motor_state.value, CCW);
  } else {
    //速度为0别切换方向
    MotorDriver_set_status(motor_controller->motor_driver, 0, motor_controller->motor_driver->dir);
  }
}

void MotorController_control(MotorController *motor_controller) {
  // 速度环
  if (motor_controller->tar_motor_state.mode == VELOCITY) {
    MotorController_control_vel(motor_controller);
  }

  motor_controller->lst_motor_state = motor_controller->cur_motor_state;
}