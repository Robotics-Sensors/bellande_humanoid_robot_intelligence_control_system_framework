/*
 * TimeStamp.h
 *
 *  Created on: 2016. 5. 16.
 *      Author: zerom
 */

#ifndef ROBOTIS_DEVICE_INCLUDE_ROBOTIS_DEVICE_TIMESTAMP_H_
#define ROBOTIS_DEVICE_INCLUDE_ROBOTIS_DEVICE_TIMESTAMP_H_


namespace ROBOTIS
{

class TimeStamp {
public:
    long sec;
    long nsec;
    TimeStamp(long sec, long nsec) : sec(sec), nsec(nsec) { }
};

}


#endif /* ROBOTIS_DEVICE_INCLUDE_ROBOTIS_DEVICE_TIMESTAMP_H_ */
