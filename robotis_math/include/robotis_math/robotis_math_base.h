/*
 * RobotisMathBase.h
 *
 *  Created on: 2016. 3. 28.
 *      Author: JaySong
 */

#ifndef ROBOTIS_MATH_BASE_H_
#define ROBOTIS_MATH_BASE_H_

#include <cmath>

namespace robotis_framework
{

#define PRINT_VAR(X) std::cout << #X << " : " << X << std::endl
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

#define deg2rad 	(M_PI / 180.0)
#define rad2deg 	(180.0 / M_PI)

inline double powDI(double a, int b)
{
	return (b == 0 ? 1 : (b > 0 ? a * powDI(a, b - 1) : 1 / powDI(a, -b)));
}

double sign( double x );

}



#endif /* ROBOTIS_MATH_BASE_H_ */
