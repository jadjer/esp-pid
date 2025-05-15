// Copyright 2025 Pavel Suprunov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <pid/PIDController.hpp>

#include <esp_timer.h>

namespace pid {

PIDController::PIDController(float _kP, float _kI, float _kD, float _maxOutput, float _maxRate) noexcept
    : kP(_kP), kI(_kI), kD(_kD), maxRate(_maxRate), maxOutput(_maxOutput), m_integral(0.0f), m_errorPrev(0.0f), m_outputPrev(0.0f) {
  m_timestampPrev = esp_timer_get_time();
}

float PIDController::compute(float error) {
  auto const currentTime = esp_timer_get_time();
  auto const dT = static_cast<float>(currentTime);
  auto const proportional = kP * error;                    //!< Пропорциональная составляющая
  auto const derivative = kD * (error - m_errorPrev) / dT; //!< Дифференциальная составляющая

  m_integral += kI * error * dT; //!< Интегральная составляющая

  auto output = proportional + m_integral + derivative; //!< Вычисление выходного сигнала на основе ошибки

  auto const outputRate = (output - m_outputPrev) / dT;
  if (std::abs(outputRate) > maxRate) {
    output = m_outputPrev + (outputRate > 0 ? maxRate : -maxRate) * dT;
  }

  // Ограничение максимального значения (clamping)
  if (output > maxOutput) {
    output = maxOutput;
  }

  if (output < -maxOutput) {
    output = -maxOutput;
  }

  // Сохраняем текущее значение выходного сигнала для rate limiting
  m_outputPrev = output;
  m_errorPrev = error;

  return output;
}

float PIDController::compute(float target, float current) {
  float error = target - current;

  return compute(error);
}

void PIDController::reset() {
  m_integral = 0.0f;
  m_outputPrev = 0.0f;
  m_errorPrev = 0.0f;
}

} // namespace pid
