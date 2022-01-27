//
// Created by crepusculumx on 2021/12/19.
//

#include "PwmDriver.h"

void PwmDriver_init(PwmDriver *pwmDriver,
                    uint16_t clock_rate,
                    TIM_HandleTypeDef *htim,
                    uint32_t channel,
                    uint16_t prescaler,
                    uint64_t max_counter_priod) {
  // var
  pwmDriver->clock_rate = clock_rate;
  pwmDriver->htim = htim;
  pwmDriver->channel = channel;
  pwmDriver->prescaler = prescaler;
  pwmDriver->max_counter_priod = max_counter_priod;

  // start
  PwmDriver_start(pwmDriver);
}

void PwmDriver_start(PwmDriver *pwm_driver) {
  HAL_TIM_PWM_Start(pwm_driver->htim, pwm_driver->channel);
}

void PwmDriver_stop(PwmDriver *pwm_driver) {
  HAL_TIM_PWM_Stop(pwm_driver->htim, pwm_driver->channel);
}

void PwmDriver_set_pwm(PwmDriver *pwm_driver, int rate, double duty_ratio) {
  pwm_driver->rate = rate;
  pwm_driver->duty_ratio = duty_ratio;

  // 占空比[0, 1]
  if (duty_ratio > 1 || duty_ratio < 0) return;

  // 特判，如果频率是0，改占空比而不是改频率
  if (rate == 0) {
        __HAL_TIM_SetCompare(pwm_driver->htim, pwm_driver->channel, 0);
    return;
  }

  // 计数器能设置的上限，判断是否能生成该频率的pwm
  uint64_t counter_priod = pwm_driver->clock_rate * 1000000 / pwm_driver->prescaler / rate;
  if (counter_priod > pwm_driver->max_counter_priod) {
    return;
  }

  pwm_driver->counter_priod = counter_priod;

  __HAL_TIM_SET_AUTORELOAD(pwm_driver->htim, counter_priod);
      __HAL_TIM_SetCompare(pwm_driver->htim,
                           pwm_driver->channel,
                           (uint16_t) ((double) counter_priod * pwm_driver->duty_ratio));
}
