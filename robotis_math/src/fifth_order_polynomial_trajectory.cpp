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
 * fifth_order_polynomial_trajectory.cpp
 *
 *  Created on: 2016. 8. 24.
 *      Author: Jay Song
 */

#include "robotis_math/fifth_order_polynomial_trajectory.h"

using namespace robotis_framework;

FifthOrderPolynomialTrajectory::FifthOrderPolynomialTrajectory(double initial_time, double initial_pos, double initial_vel, double initial_acc,
                                                               double final_time,   double final_pos,   double final_vel,   double final_acc)
{
  position_coeff_.resize(6, 1);
  velocity_coeff_.resize(6, 1);
  acceleration_coeff_.resize(6, 1);
  time_variables_.resize(1, 6);

  position_coeff_.fill(0);
  velocity_coeff_.fill(0);
  acceleration_coeff_.fill(0);
  time_variables_.fill(0);

  if(final_time > initial_time)
  {
    initial_time_ = initial_time;
    initial_pos_  = initial_pos;
    initial_vel_  = initial_vel;
    initial_acc_  = initial_acc;

    current_time_ = initial_time;
    current_pos_  = initial_pos;
    current_vel_  = initial_vel;
    current_acc_  = initial_acc;

    final_time_ = final_time;
    final_pos_  = final_pos;
    final_vel_  = final_vel;
    final_acc_  = final_acc;

    Eigen::MatrixXd time_mat;
    Eigen::MatrixXd conditions_mat;

    time_mat.resize(6,6);
    time_mat <<  powDI(initial_time_, 5),     powDI(initial_time_, 4),      powDI(initial_time_, 3), powDI(initial_time_, 2), initial_time_, 1.0,
             5.0*powDI(initial_time_, 4),  4.0*powDI(initial_time_, 3), 3.0*powDI(initial_time_, 2),    2.0*initial_time_,              1.0, 0.0,
            20.0*powDI(initial_time_, 3), 12.0*powDI(initial_time_, 2), 6.0*initial_time_,              2.0,                            0.0, 0.0,
                 powDI(final_time_, 5),      powDI(final_time_, 4),     powDI(final_time_, 3), powDI(final_time_, 2), final_time_, 1.0,
             5.0*powDI(final_time_, 4),  4.0*powDI(final_time_, 3), 3.0*powDI(final_time_, 2),    2.0*final_time_,            1.0, 0.0,
            20.0*powDI(final_time_, 3), 12.0*powDI(final_time_, 2), 6.0*final_time_,              2.0,                        0.0, 0.0;

    conditions_mat.resize(6,1);
    conditions_mat << initial_pos_, initial_vel_, initial_acc_, final_pos_, final_vel_, final_acc_;

    position_coeff_ = time_mat.inverse() * conditions_mat;
    velocity_coeff_ <<                            0.0,
                       5.0*position_coeff_.coeff(0,0),
                       4.0*position_coeff_.coeff(1,0),
                       3.0*position_coeff_.coeff(2,0),
                       2.0*position_coeff_.coeff(3,0),
                       1.0*position_coeff_.coeff(4,0);
    acceleration_coeff_ <<                            0.0,
                                                      0.0,
                          20.0*position_coeff_.coeff(0,0),
                          12.0*position_coeff_.coeff(1,0),
                           6.0*position_coeff_.coeff(2,0),
                           2.0*position_coeff_.coeff(3,0);
  }

}

FifthOrderPolynomialTrajectory::FifthOrderPolynomialTrajectory()
{
  initial_time_ = 0;
  initial_pos_  = 0;
  initial_vel_  = 0;
  initial_acc_  = 0;

  current_time_ = 0;
  current_pos_  = 0;
  current_vel_  = 0;
  current_acc_  = 0;

  final_time_ = 0;
  final_pos_  = 0;
  final_vel_  = 0;
  final_acc_  = 0;

  position_coeff_.resize(6, 1);
  velocity_coeff_.resize(6, 1);
  acceleration_coeff_.resize(6, 1);
  time_variables_.resize(1, 6);

  position_coeff_.fill(0);
  velocity_coeff_.fill(0);
  acceleration_coeff_.fill(0);
  time_variables_.fill(0);
}

FifthOrderPolynomialTrajectory::~FifthOrderPolynomialTrajectory()
{

}

bool FifthOrderPolynomialTrajectory::changeTrajectory(double final_pos,   double final_vel,   double final_acc)
{
  final_pos_  = final_pos;
  final_vel_  = final_vel;
  final_acc_  = final_acc;

  Eigen::MatrixXd time_mat;
  Eigen::MatrixXd conditions_mat;

  time_mat.resize(6,6);
  time_mat <<  powDI(initial_time_, 5),     powDI(initial_time_, 4),      powDI(initial_time_, 3), powDI(initial_time_, 2), initial_time_, 1.0,
           5.0*powDI(initial_time_, 4),  4.0*powDI(initial_time_, 3), 3.0*powDI(initial_time_, 2),    2.0*initial_time_,              1.0, 0.0,
          20.0*powDI(initial_time_, 3), 12.0*powDI(initial_time_, 2), 6.0*initial_time_,              2.0,                            0.0, 0.0,
               powDI(final_time_, 5),      powDI(final_time_, 4),     powDI(final_time_, 3), powDI(final_time_, 2), final_time_, 1.0,
           5.0*powDI(final_time_, 4),  4.0*powDI(final_time_, 3), 3.0*powDI(final_time_, 2),    2.0*final_time_,            1.0, 0.0,
          20.0*powDI(final_time_, 3), 12.0*powDI(final_time_, 2), 6.0*final_time_,              2.0,                        0.0, 0.0;

  conditions_mat.resize(6,1);
  conditions_mat << initial_pos_, initial_vel_, initial_acc_, final_pos_, final_vel_, final_acc_;

  position_coeff_ = time_mat.inverse() * conditions_mat;
  velocity_coeff_ <<                            0.0,
                     5.0*position_coeff_.coeff(0,0),
                     4.0*position_coeff_.coeff(1,0),
                     3.0*position_coeff_.coeff(2,0),
                     2.0*position_coeff_.coeff(3,0),
                     1.0*position_coeff_.coeff(4,0);
  acceleration_coeff_ <<                            0.0,
                                                    0.0,
                         20.0*position_coeff_.coeff(0,0),
                         12.0*position_coeff_.coeff(1,0),
                          6.0*position_coeff_.coeff(2,0),
                          2.0*position_coeff_.coeff(3,0);

  return true;
}

bool FifthOrderPolynomialTrajectory::changeTrajectory(double final_time,   double final_pos,   double final_vel,   double final_acc)
{
  if(final_time < initial_time_)
    return false;

  final_time_ = final_time;
  return changeTrajectory(final_pos, final_vel, final_acc);
}

bool FifthOrderPolynomialTrajectory::changeTrajectory(double initial_time, double initial_pos, double initial_vel, double initial_acc,
                                                      double final_time,   double final_pos,   double final_vel,   double final_acc)
{
  if(final_time < initial_time)
    return false;

  initial_time_ = initial_time;
  initial_pos_  = initial_pos;
  initial_vel_  = initial_vel;
  initial_acc_  = initial_acc;

  final_time_ = final_time;

  return changeTrajectory(final_pos, final_vel, final_acc);
}

double FifthOrderPolynomialTrajectory::getPosition(double time)
{
  if(time >= final_time_)
  {
    current_time_ = final_time_;
    current_pos_  = final_pos_;
    current_vel_  = final_vel_;
    current_acc_  = final_acc_;
    return final_pos_;
  }
  else if(time <= initial_time_ )
  {
    current_time_ = initial_time_;
    current_pos_  = initial_pos_;
    current_vel_  = initial_vel_;
    current_acc_  = initial_acc_;
    return initial_pos_;
  }
  else
  {
    current_time_ = time;
    time_variables_ << powDI(time, 5), powDI(time, 4), powDI(time, 3), powDI(time, 2), time, 1.0;
    current_pos_ = (time_variables_ * position_coeff_).coeff(0,0);
    current_vel_ = (time_variables_ * velocity_coeff_).coeff(0,0);
    current_acc_ = (time_variables_ * acceleration_coeff_).coeff(0,0);
    return current_pos_;
  }
}

double FifthOrderPolynomialTrajectory::getVelocity(double time)
{
  if(time >= final_time_)
  {
    current_time_ = final_time_;
    current_pos_  = final_pos_;
    current_vel_  = final_vel_;
    current_acc_  = final_acc_;
    return final_vel_;
  }
  else if(time <= initial_time_ )
  {
    current_time_ = initial_time_;
    current_pos_  = initial_pos_;
    current_vel_  = initial_vel_;
    current_acc_  = initial_acc_;
    return initial_vel_;
  }
  else
  {
    current_time_ = time;
    time_variables_ << powDI(time, 5), powDI(time, 4), powDI(time, 3), powDI(time, 2), time, 1.0;
    current_pos_ = (time_variables_ * position_coeff_).coeff(0,0);
    current_vel_ = (time_variables_ * velocity_coeff_).coeff(0,0);
    current_acc_ = (time_variables_ * acceleration_coeff_).coeff(0,0);
    return current_vel_;
  }
}

double FifthOrderPolynomialTrajectory::getAcceleration(double time)
{
  if(time >= final_time_)
  {
    current_time_ = final_time_;
    current_pos_  = final_pos_;
    current_vel_  = final_vel_;
    current_acc_  = final_acc_;
    return final_acc_;
  }
  else if(time <= initial_time_ )
  {
    current_time_ = initial_time_;
    current_pos_  = initial_pos_;
    current_vel_  = initial_vel_;
    current_acc_  = initial_acc_;
    return initial_acc_;
  }
  else
  {
    current_time_ = time;
    time_variables_ << powDI(time, 5), powDI(time, 4), powDI(time, 3), powDI(time, 2), time, 1.0;
    current_pos_ = (time_variables_ * position_coeff_).coeff(0,0);
    current_vel_ = (time_variables_ * velocity_coeff_).coeff(0,0);
    current_acc_ = (time_variables_ * acceleration_coeff_).coeff(0,0);
    return current_acc_;
  }
}

void FifthOrderPolynomialTrajectory::setTime(double time)
{
  if(time >= final_time_)
  {
    current_time_ = final_time_;
    current_pos_  = final_pos_;
    current_vel_  = final_vel_;
    current_acc_  = final_acc_;
  }
  else if(time <= initial_time_ )
  {
    current_time_ = initial_time_;
    current_pos_  = initial_pos_;
    current_vel_  = initial_vel_;
    current_acc_  = initial_acc_;
  }
  else
  {
    current_time_ = time;
    time_variables_ << powDI(time, 5), powDI(time, 4), powDI(time, 3), powDI(time, 2), time, 1.0;
    current_pos_ = (time_variables_ * position_coeff_).coeff(0,0);
    current_vel_ = (time_variables_ * velocity_coeff_).coeff(0,0);
    current_acc_ = (time_variables_ * acceleration_coeff_).coeff(0,0);
  }
}

double FifthOrderPolynomialTrajectory::getPosition()
{
  return current_pos_;
}

double FifthOrderPolynomialTrajectory::getVelocity()
{
  return current_vel_;
}

double FifthOrderPolynomialTrajectory::getAcceleration()
{
  return current_acc_;
}
