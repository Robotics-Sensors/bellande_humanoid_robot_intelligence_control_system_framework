/*
 * RobotisMathBase.cpp
 *
 *  Created on: Mar 18, 2016
 *      Author: jay
 */

#include "robotis_math/RobotisMathBase.h"




namespace ROBOTIS
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
