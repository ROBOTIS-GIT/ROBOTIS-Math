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

/*
 * robotis_trajectory_calculator.h
 *
 *  Created on: June 7, 2016
 *      Author: sch
 */

#ifndef ROBOTIS_MATH_ROBOTIS_TRAJECTORY_CALCULATOR_H_
#define ROBOTIS_MATH_ROBOTIS_TRAJECTORY_CALCULATOR_H_

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include "robotis_linear_algebra.h"
#include "robotis_math_base.h"
#include "fifth_order_polynomial_trajectory.h"
#include "simple_trapezoidal_velocity_profile.h"

namespace robotis_framework
{
// minimum jerk trajectory
Eigen::MatrixXd calcMinimumJerkTra(double pos_start, double vel_start, double accel_start,
                                   double pos_end,   double vel_end,   double accel_end,
                                   double smp_time,  double mov_time);

Eigen::MatrixXd calcMinimumJerkTraPlus(double pos_start, double vel_start, double accel_start,
                                       double pos_end,   double vel_end,   double accel_end,
                                       double smp_time,  double mov_time);

Eigen::MatrixXd calcMinimumJerkTraWithViaPoints(int via_num,
                                                double pos_start, double vel_start, double accel_start,
                                                Eigen::MatrixXd pos_via,  Eigen::MatrixXd vel_via, Eigen::MatrixXd accel_via,
                                                double pos_end, double vel_end, double accel_end,
                                                double smp_time, Eigen::MatrixXd via_time, double mov_time);

Eigen::MatrixXd calcMinimumJerkTraWithViaPointsPosition(int via_num,
                                                        double pos_start, double vel_start, double accel_start,
                                                        Eigen::MatrixXd pos_via,
                                                        double pos_end, double vel_end, double accel_end,
                                                        double smp_time, Eigen::MatrixXd via_time, double mov_time);

Eigen::MatrixXd calcArc3dTra(double smp_time, double mov_time,
                             Eigen::MatrixXd center_point, Eigen::MatrixXd normal_vector, Eigen::MatrixXd start_point,
                             double rotation_angle, double cross_ratio);

}

#endif /* ROBOTIS_MATH_ROBOTIS_TRAJECTORY_CALCULATOR_H_ */
