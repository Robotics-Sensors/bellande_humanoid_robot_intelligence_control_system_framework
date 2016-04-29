/*
 * RobotisTrajectoryCalculator.h
 *
 *  Created on: Mar 18, 2016
 *      Author: jay
 */

#ifndef ROBOTIS_TRAJECTORY_CALCULATOR_H_
#define ROBOTIS_TRAJECTORY_CALCULATOR_H_


#include "RobotisMathBase.h"
#include "RobotisLinearAlgebra.h"

// minimum jerk trajectory

namespace ROBOTIS
{

Eigen::MatrixXd minimum_jerk_tra( double pos_start , double vel_start , double accel_start,
                                  double pos_end ,   double vel_end ,   double accel_end,
                                  double smp_time ,  double mov_time );

Eigen::MatrixXd minimum_jerk_tra_via_n_qdqddq( int via_num,
                                              double pos_start , double vel_start , double accel_start ,
                                              Eigen::MatrixXd pos_via,  Eigen::MatrixXd vel_via, Eigen::MatrixXd accel_via,
                                              double pos_end, double vel_end, double accel_end,
                                              double smp_time, Eigen::MatrixXd via_time, double mov_time );

Eigen::MatrixXd arc3d_tra( double smp_time, double mov_time,
                           Eigen::MatrixXd center_point, Eigen::MatrixXd normal_vector, Eigen::MatrixXd start_point,
                           double rotation_angle, double cross_ratio );

}


#endif /* ROBOTIS_TRAJECTORY_CALCULATOR_H_ */
