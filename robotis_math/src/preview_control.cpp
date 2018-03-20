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

#include "robotis_math/preview_control.h"

using namespace robotis_framework;

PreviewControl::PreviewControl()
{

}

PreviewControl::~PreviewControl()
{

}

Eigen::MatrixXd PreviewControl::calcPreviewParam(double preview_time, double control_cycle,
                                                 double lipm_height,
                                                 Eigen::MatrixXd K, Eigen::MatrixXd P)
{
  double t = control_cycle;
  double preview_size_ = round(preview_time/control_cycle) + 1;

  Eigen::MatrixXd A;
  A.resize(3,3);
  A << 1,  t,  t*t/2.0,
        0,  1,  t,
        0,  0,  1;

  Eigen::MatrixXd b;
  b.resize(3,1);
  b << t*t*t/6.0,
        t*t/2.0,
        t;

  Eigen::MatrixXd c_;
  c_.resize(1,3);
  c_ << 1, 0, -lipm_height/9.81;

  Eigen::MatrixXd tempA = Eigen::MatrixXd::Zero(4,4);
  Eigen::MatrixXd tempb = Eigen::MatrixXd::Zero(4,1);
  Eigen::MatrixXd tempc = Eigen::MatrixXd::Zero(1,4);

  tempA.coeffRef(0,0) = 1;
  tempA.block<1,3>(0,1) = c_*A;
  tempA.block<3,3>(1,1) = A;

  tempb.coeffRef(0,0) = (c_*b).coeff(0,0);
  tempb.block<3,1>(1,0) = b;

  tempc.coeffRef(0,0) = 1;

  double R = 1e-6;
  double Q_e = 1;
  double Q_x = 0;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4,4);
  Q.coeffRef(0,0) = Q_e;
  Q.coeffRef(1,1) = Q_e;
  Q.coeffRef(2,2) = Q_e;
  Q.coeffRef(3,3) = Q_x;

  Eigen::MatrixXd f;
  f.resize(1, preview_size_);

  Eigen::MatrixXd mat_R = Eigen::MatrixXd::Zero(1,1);
  mat_R.coeffRef(0,0) = R;

  Eigen::MatrixXd tempCoeff1 = mat_R + ((tempb.transpose() * P) * tempb);
  Eigen::MatrixXd tempCoeff1_inv = tempCoeff1.inverse();
  Eigen::MatrixXd tempCoeff2 = tempb.transpose();
  Eigen::MatrixXd tempCoeff3 = Eigen::MatrixXd::Identity(4,4);
  Eigen::MatrixXd tempCoeff4 = P*tempc.transpose();

  f.block<1,1>(0,0) = ((tempCoeff1_inv*tempCoeff2)* tempCoeff3) * tempCoeff4;

  for(int i = 1; i < preview_size_; i++)
  {
    tempCoeff3 = tempCoeff3*((tempA - tempb*K).transpose());
    f.block<1,1>(0,i) = ((tempCoeff1_inv*tempCoeff2)* tempCoeff3) * tempCoeff4;
  }

  return f;
}
