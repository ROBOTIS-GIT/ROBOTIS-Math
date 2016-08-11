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
 * robotis_linear_algebra.cpp
 *
 *  Created on: June 7, 2016
 *      Author: sch
 */

#include "robotis_math/robotis_linear_algebra.h"

namespace robotis_framework
{

Eigen::MatrixXd getTransitionXYZ(double position_x, double position_y, double position_z)
{
  Eigen::MatrixXd position(3,1);

  position <<
      position_x,
      position_y,
      position_z;

  return position;
}

Eigen::MatrixXd getTransformationXYZRPY(double position_x, double position_y, double position_z , double roll , double pitch , double yaw)
{
  Eigen::MatrixXd transformation = getRotation4d(roll, pitch, yaw);
  transformation.coeffRef(0,3) = position_x;
  transformation.coeffRef(1,3) = position_y;
  transformation.coeffRef(2,3) = position_z;

  return transformation;
}

Eigen::MatrixXd getInverseTransformation(Eigen::MatrixXd transform)
{
  // If T is Transform Matrix A from B, the BOA is translation component coordi. B to coordi. A

  Eigen::Vector3d vec_boa;
  Eigen::Vector3d vec_x, vec_y, vec_z;
  Eigen::MatrixXd inv_t(4,4);

  vec_boa(0) = -transform(0,3);
  vec_boa(1) = -transform(1,3);
  vec_boa(2) = -transform(2,3);

  vec_x(0) = transform(0,0); vec_x(1) = transform(1,0); vec_x(2) = transform(2,0);
  vec_y(0) = transform(0,1); vec_y(1) = transform(1,1); vec_y(2) = transform(2,1);
  vec_z(0) = transform(0,2); vec_z(1) = transform(1,2); vec_z(2) = transform(2,2);

  inv_t <<
      vec_x(0), vec_x(1), vec_x(2), vec_boa.dot(vec_x),
      vec_y(0), vec_y(1), vec_y(2), vec_boa.dot(vec_y),
      vec_z(0), vec_z(1), vec_z(2), vec_boa.dot(vec_z),
      0, 0, 0, 1;

  return inv_t;
}

Eigen::MatrixXd getInertiaXYZ(double ixx, double ixy, double ixz , double iyy , double iyz, double izz)
{
  Eigen::MatrixXd inertia(3,3);

  inertia <<
      ixx, ixy, ixz,
      ixy, iyy, iyz,
      ixz, iyz, izz;

  return inertia;
}

Eigen::MatrixXd getRotationX(double angle)
{
  Eigen::MatrixXd rotation(3,3);

  rotation <<
      1.0, 0.0, 0.0,
      0.0, cos(angle), -sin(angle),
      0.0, sin(angle), cos(angle);

  return rotation;
}

Eigen::MatrixXd getRotationY(double angle)
{
  Eigen::MatrixXd rotation(3,3);

  rotation <<
      cos(angle), 0.0, sin(angle),
      0.0, 1.0, 0.0,
      -sin(angle), 0.0, cos(angle);

  return rotation;
}

Eigen::MatrixXd getRotationZ(double angle)
{
  Eigen::MatrixXd rotation(3,3);

  rotation <<
      cos(angle), -sin(angle), 0.0,
      sin(angle), cos(angle), 0.0,
      0.0, 0.0, 1.0;

  return rotation;
}

Eigen::MatrixXd getRotation4d(double roll, double pitch, double yaw )
{
  double sr = sin(roll), cr = cos(roll);
  double sp = sin(pitch), cp = cos(pitch);
  double sy = sin(yaw), cy = cos(yaw);

  Eigen::MatrixXd mat_roll(4,4);
  Eigen::MatrixXd mat_pitch(4,4);
  Eigen::MatrixXd mat_yaw(4,4);

  mat_roll <<
      1, 0, 0, 0,
      0, cr, -sr, 0,
      0, sr, cr, 0,
      0, 0, 0, 1;

  mat_pitch <<
      cp, 0, sp, 0,
      0, 1, 0, 0,
      -sp, 0, cp, 0,
      0, 0, 0, 1;

  mat_yaw <<
      cy, -sy, 0, 0,
      sy, cy, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;

  Eigen::MatrixXd mat_rpy = (mat_yaw*mat_pitch)*mat_roll;

  return mat_rpy;
}

Eigen::MatrixXd getTranslation4D(double position_x, double position_y, double position_z)
{
  Eigen::MatrixXd mat_translation(4,4);

  mat_translation <<
      1, 0, 0, position_x,
      0, 1, 0, position_y,
      0, 0, 1, position_z,
      0, 0, 0,          1;

  return mat_translation;
}

Eigen::MatrixXd convertRotationToRPY(Eigen::MatrixXd rotation)
{
  Eigen::MatrixXd rpy = Eigen::MatrixXd::Zero(3,1);

  rpy.coeffRef(0,0) = atan2(rotation.coeff(2,1), rotation.coeff(2,2));
  rpy.coeffRef(1,0) = atan2(-rotation.coeff(2,0), sqrt(pow(rotation.coeff(2,1), 2) + pow(rotation.coeff(2,2),2)));
  rpy.coeffRef(2,0) = atan2 (rotation.coeff(1,0), rotation.coeff(0,0));

  return rpy;
}

Eigen::MatrixXd convertRPYToRotation(double roll, double pitch, double yaw)
{
  Eigen::MatrixXd rotation = getRotationZ(yaw)*getRotationY(pitch)*getRotationX(roll);

  return rotation;
}

Eigen::Quaterniond convertRPYToQuaternion(double roll, double pitch, double yaw)
{
  Eigen::MatrixXd rotation = convertRPYToRotation(roll,pitch,yaw);

  Eigen::Matrix3d rotation3d;
  rotation3d = rotation.block<3,3>(0,0);

  Eigen::Quaterniond quaternion;
  quaternion = rotation3d;

  return quaternion;
}

Eigen::Quaterniond convertRotationToQuaternion(Eigen::MatrixXd rotation)
{
  Eigen::Matrix3d rotation3d;
  rotation3d = rotation.block<3,3>(0,0);

  Eigen::Quaterniond quaternion;
  quaternion = rotation3d;

  return quaternion;
}

Eigen::MatrixXd convertQuaternionToRPY(Eigen::Quaterniond quaternion)
{
  Eigen::MatrixXd rpy = convertRotationToRPY(quaternion.toRotationMatrix());

  return rpy;
}

Eigen::MatrixXd convertQuaternionToRotation(Eigen::Quaterniond quaternion)
{
  Eigen::MatrixXd rotation = quaternion.toRotationMatrix();

  return rotation;
}

Eigen::MatrixXd calcHatto(Eigen::MatrixXd matrix3d)
{
  Eigen::MatrixXd hatto(3,3);

  hatto <<
      0.0, -matrix3d.coeff(2,0), matrix3d.coeff(1,0),
      matrix3d.coeff(2,0), 0.0, -matrix3d.coeff(0,0),
      -matrix3d.coeff(1,0), matrix3d.coeff(0,0), 0.0;

  return hatto;
}

Eigen::MatrixXd calcRodrigues(Eigen::MatrixXd hat_matrix, double angle)
{
  Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(3,3);
  Eigen::MatrixXd rodrigues = identity+hat_matrix*sin(angle)+hat_matrix*hat_matrix*(1-cos(angle));

  return rodrigues;
}

Eigen::MatrixXd convertRotToOmega(Eigen::MatrixXd rotation)
{
  double eps = 1e-12;

  double alpha = (rotation.coeff(0,0)+rotation.coeff(1,1)+rotation.coeff(2,2)-1.0)/2.0;
  double alpha_dash = fabs( alpha - 1.0 );

  Eigen::MatrixXd rot_to_omega(3,1);

  if( alpha_dash < eps )
  {
    rot_to_omega <<
        0.0,
        0.0,
        0.0;
  }
  else
  {
    double theta = acos(alpha);

    rot_to_omega <<
        rotation.coeff(2,1)-rotation.coeff(1,2),
        rotation.coeff(0,2)-rotation.coeff(2,0),
        rotation.coeff(1,0)-rotation.coeff(0,1);

    rot_to_omega = 0.5*(theta/sin(theta))*rot_to_omega;
  }

  return rot_to_omega;
}

Eigen::MatrixXd calcCross(Eigen::MatrixXd vector3d_a, Eigen::MatrixXd vector3d_b)
{
  Eigen::MatrixXd cross(3,1);

  cross <<
      vector3d_a.coeff(1,0)*vector3d_b.coeff(2,0)-vector3d_a.coeff(2,0)*vector3d_b.coeff(1,0),
      vector3d_a.coeff(2,0)*vector3d_b.coeff(0,0)-vector3d_a.coeff(0,0)*vector3d_b.coeff(2,0),
      vector3d_a.coeff(0,0)*vector3d_b.coeff(1,0)-vector3d_a.coeff(1,0)*vector3d_b.coeff(0,0);

  return cross;
}

double calcInner(Eigen::MatrixXd vector3d_a, Eigen::MatrixXd vector3d_b)
{
  return vector3d_a.dot(vector3d_b);
}

Pose3D getPose3DfromTransformMatrix(Eigen::MatrixXd transform)
{
  Pose3D pose_3d;

  pose_3d.x     = transform.coeff(0, 3);
  pose_3d.y     = transform.coeff(1, 3);
  pose_3d.z     = transform.coeff(2, 3);
  pose_3d.roll  = atan2( transform.coeff(2,1), transform.coeff(2,2));
  pose_3d.pitch = atan2(-transform.coeff(2,0), sqrt(transform.coeff(2,1)*transform.coeff(2,1) + transform.coeff(2,2)*transform.coeff(2,2)) );
  pose_3d.yaw   = atan2( transform.coeff(1,0), transform.coeff(0,0));

  return pose_3d;
}

}
