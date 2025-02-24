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

#pragma once

#include <algorithm>
#include <iostream>

/**
 * @namespace pid
 */
namespace pid {

/**
 * @class PIDController
 * @brief PID Controller
 */
class PIDController {
public:
  using Time = std::int64_t;

public:
  /**
   * Constructor
   * @param P - Proportional gain
   * @param I - Integral gain
   * @param D - Derivative gain
   * @param ramp - Maximum speed of change of the output value
   * @param limit - Maximum output value
   */
  PIDController(float kP, float kI, float kD, float maxOutput, float maxRate);

public:
  float compute(float error);
  float compute(float target, float current);

public:
  void reset();

public:
  float const kP;       //!< Proportional gain
  float const kI;       //!< Integral gain
  float const kD;       //!< Derivative gain
  float const maxRate;  //!< Maximum speed of change of the output value
  float const maxOutput;//!< Maximum output value

private:
  Time m_timestampPrev;//!< Last execution timestamp

private:
  float m_integral;  //!< last integral component value
  float m_errorPrev; //!< last tracking error value
  float m_outputPrev;//!< last pid output value
};

}// namespace pid
