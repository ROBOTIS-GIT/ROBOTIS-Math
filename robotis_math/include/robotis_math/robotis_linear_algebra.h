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

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include <cmath>

#include <Eigen/Dense>
#include "step_data_define.h"

namespace robotis_framework
{

Eigen::Vector3d getTransitionXYZ(double position_x, double position_y, double position_z);
Eigen::Matrix4d getTransformationXYZRPY(double position_x, double position_y, double position_z , double roll, double pitch, double yaw);
Eigen::Matrix4d getInverseTransformation(const Eigen::MatrixXd& transform);
Eigen::Matrix3d getInertiaXYZ(double ixx, double ixy, double ixz , double iyy , double iyz, double izz);
Eigen::Matrix3d getRotationX(double angle);
Eigen::Matrix3d getRotationY(double angle);
Eigen::Matrix3d getRotationZ(double angle);
Eigen::Matrix4d getRotation4d(double roll, double pitch, double yaw);
Eigen::Matrix4d getTranslation4D(double position_x, double position_y, double position_z);

Eigen::Vector3d convertRotationToRPY(const Eigen::Matrix3d& rotation);
Eigen::Matrix3d convertRPYToRotation(double roll, double pitch, double yaw);
Eigen::Quaterniond convertRPYToQuaternion(double roll, double pitch, double yaw);
Eigen::Quaterniond convertRotationToQuaternion(const Eigen::Matrix3d& rotation);
Eigen::Vector3d convertQuaternionToRPY(const Eigen::Quaterniond& quaternion);
Eigen::Matrix3d convertQuaternionToRotation(const Eigen::Quaterniond& quaternion);

Eigen::Matrix3d calcHatto(const Eigen::Vector3d& matrix3d);
Eigen::Matrix3d calcRodrigues(const Eigen::Matrix3d& hat_matrix, double angle);
Eigen::Vector3d convertRotToOmega(const Eigen::Matrix3d& rotation);
Eigen::Vector3d calcCross(const Eigen::Vector3d& vector3d_a, const Eigen::Vector3d& vector3d_b);
double calcInner(const Eigen::MatrixXd& vector3d_a, const Eigen::MatrixXd& vector3d_b);

Pose3D getPose3DfromTransformMatrix(const Eigen::Matrix4d& transform);

}



#endif /* ROBOTIS_MATH_ROBOTIS_LINEAR_ALGEBRA_H_ */
