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
 * robotis_linear_algebra.h
 *
 *  Created on: June 7, 2016
 *      Author: sch
 */

#ifndef ROBOTIS_MATH_ROBOTIS_LINEAR_ALGEBRA_H_
#define ROBOTIS_MATH_ROBOTIS_LINEAR_ALGEBRA_H_

#include <cmath>

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include <Eigen/Dense>
#include "step_data_define.h"

namespace robotis_framework
{

Eigen::MatrixXd getTransitionXYZ(double position_x, double position_y, double position_z);
Eigen::MatrixXd getTransformationXYZRPY(double position_x, double position_y, double position_z , double roll, double pitch, double yaw);
Eigen::MatrixXd getInverseTransformation(Eigen::MatrixXd transform);
Eigen::MatrixXd getInertiaXYZ(double ixx, double ixy, double ixz , double iyy , double iyz, double izz);
Eigen::MatrixXd getRotationX(double angle);
Eigen::MatrixXd getRotationY(double angle);
Eigen::MatrixXd getRotationZ(double angle);
Eigen::MatrixXd getRotation4d(double roll, double pitch, double yaw);
Eigen::MatrixXd getTranslation4D(double position_x, double position_y, double position_z);

Eigen::MatrixXd convertRotationToRPY(Eigen::MatrixXd rotation);
Eigen::MatrixXd convertRPYToRotation(double roll, double pitch, double yaw);
Eigen::Quaterniond convertRPYToQuaternion(double roll, double pitch, double yaw);
Eigen::Quaterniond convertRotationToQuaternion(Eigen::MatrixXd rotation);
Eigen::MatrixXd convertQuaternionToRPY(Eigen::Quaterniond quaternion);
Eigen::MatrixXd convertQuaternionToRotation(Eigen::Quaterniond quaternion);

Eigen::MatrixXd calcHatto(Eigen::MatrixXd matrix3d);
Eigen::MatrixXd calcRodrigues(Eigen::MatrixXd hat_matrix, double angle);
Eigen::MatrixXd convertRotToOmega(Eigen::MatrixXd rotation);
Eigen::MatrixXd calcCross(Eigen::MatrixXd vector3d_a, Eigen::MatrixXd vector3d_b);
double calcInner(Eigen::MatrixXd vector3d_a, Eigen::MatrixXd vector3d_b);

Pose3D getPose3DfromTransformMatrix(Eigen::MatrixXd  transform);

}



#endif /* ROBOTIS_MATH_ROBOTIS_LINEAR_ALGEBRA_H_ */
