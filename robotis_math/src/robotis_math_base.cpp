/*
 * RobotisMathBase.cpp
 *
 *  Created on: Mar 18, 2016
 *      Author: jay
 */

#include "../include/robotis_math/robotis_math_base.h"




namespace robotis_framework
{

double sign( double x )
{
    if ( x < 0.0 )
		return -1.0;
	else if ( x > 0.0)
		return 1.0;
	else
		return 0.0;
}

}
