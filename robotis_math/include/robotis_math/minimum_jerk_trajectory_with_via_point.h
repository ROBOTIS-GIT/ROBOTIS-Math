/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#ifndef ROBOTIS_MATH_MINIMUM_JERK_TRAJECTORY_WITH_VIA_POINT_H_
#define ROBOTIS_MATH_MINIMUM_JERK_TRAJECTORY_WITH_VIA_POINT_H_

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include "robotis_linear_algebra.h"
#include "robotis_math_base.h"

#include <ros/ros.h>
#include <stdint.h>
#include <vector>

namespace robotis_framework
{

class MinimumJerkViaPoint
{
public:
  MinimumJerkViaPoint(double ini_time, double fin_time, double via_time, double ratio,
                      std::vector<double_t> ini_pos, std::vector<double_t> ini_vel, std::vector<double_t> ini_acc,
                      std::vector<double_t> fin_pos, std::vector<double_t> fin_vel, std::vector<double_t> fin_acc,
                      std::vector<double_t> via_pos, std::vector<double_t> via_vel, std::vector<double_t> via_acc);
  virtual ~MinimumJerkViaPoint();

  std::vector<double_t> getPosition(double time);
  std::vector<double_t> getVelocity(double time);
  std::vector<double_t> getAcceleration(double time);

  double cur_time_;
  std::vector<double_t> cur_pos_;
  std::vector<double_t> cur_vel_;
  std::vector<double_t> cur_acc_;

  Eigen::MatrixXd position_coeff_;
  Eigen::MatrixXd velocity_coeff_;
  Eigen::MatrixXd acceleration_coeff_;
  Eigen::MatrixXd time_variables_;

private:
  int number_of_joint_;
  double ratio_;
  double input_ini_time_, input_fin_time_;
  double ini_time_, fin_time_, via_time_;
  std::vector<double_t> ini_pos_, ini_vel_, ini_acc_;
  std::vector<double_t> fin_pos_, fin_vel_, fin_acc_;
  std::vector<double_t> via_pos_, via_vel_, via_acc_;
};

}

#endif /* ROBOTIS_MATH_MINIMUM_JERK_TRAJECTORY_WITH_VIA_POINT_H_ */
