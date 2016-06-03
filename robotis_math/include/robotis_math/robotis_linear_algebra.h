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
 *  Created on: Mar 18, 2016
 *      Author: jay
 */

#ifndef ROBOTIS_LINEAR_ALGEBRA_H_
#define ROBOTIS_LINEAR_ALGEBRA_H_

#include <cmath>

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include <Eigen/Dense>

namespace robotis_framework
{

Eigen::MatrixXd transitionXYZ ( double position_x, double position_y, double position_z );
Eigen::MatrixXd transformationXYZRPY ( double position_x, double position_y, double position_z , double roll , double pitch , double yaw );
Eigen::MatrixXd InverseTransformation(Eigen::MatrixXd transform);

Eigen::MatrixXd inertiaXYZ( double ixx, double ixy, double ixz , double iyy , double iyz, double izz );

Eigen::MatrixXd rotationX( double angle );
Eigen::MatrixXd rotationY( double angle );
Eigen::MatrixXd rotationZ( double angle );

Eigen::MatrixXd rotation2rpy( Eigen::MatrixXd rotation );
Eigen::MatrixXd rpy2rotation( double roll, double pitch, double yaw );

Eigen::Quaterniond rpy2quaternion( double roll, double pitch, double yaw );
Eigen::Quaterniond rotation2quaternion( Eigen::MatrixXd rotation );

Eigen::MatrixXd quaternion2rpy( Eigen::Quaterniond quaternion );
Eigen::MatrixXd quaternion2rotation( Eigen::Quaterniond quaternion );

Eigen::MatrixXd rotation4d( double roll, double pitch, double yaw );

Eigen::MatrixXd hatto( Eigen::MatrixXd matrix3d );
Eigen::MatrixXd Rodrigues( Eigen::MatrixXd hat_matrix , double angle );
Eigen::MatrixXd rot2omega(Eigen::MatrixXd rotation );
Eigen::MatrixXd cross(Eigen::MatrixXd vector3d_a, Eigen::MatrixXd vector3d_b );
double dot(Eigen::MatrixXd vector3d_a, Eigen::MatrixXd vector3d_b );

}



#endif /* ROBOTIS_LINEAR_ALGEBRA_H_ */
