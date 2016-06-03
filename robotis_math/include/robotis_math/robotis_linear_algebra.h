/*
 * RobotisLinearAlgebra.h
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
