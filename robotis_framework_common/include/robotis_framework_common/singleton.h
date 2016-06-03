/*
 * Singleton.h
 *
 *  Created on: 2016. 5. 17.
 *      Author: zerom
 */

#ifndef ROBOTIS_FRAMEWORK_COMMON_INCLUDE_ROBOTIS_FRAMEWORK_COMMON_SINGLETON_H_
#define ROBOTIS_FRAMEWORK_COMMON_INCLUDE_ROBOTIS_FRAMEWORK_COMMON_SINGLETON_H_


namespace ROBOTIS
{

template <class T>
class Singleton
{
private:
    static T *unique_instance_;

protected:
    Singleton() { }
    Singleton(Singleton const&) { }
    Singleton& operator=(Singleton const&) { return *this; }

public:
    static T* GetInstance()
    {
        if(unique_instance_ == NULL)
            unique_instance_ = new T;
        return unique_instance_;
    }

    static void DestroyInstance()
    {
        if(unique_instance_)
        {
            delete unique_instance_;
            unique_instance_ = NULL;
        }
    }
};

template <class T> T* Singleton<T>::unique_instance_ = NULL;

}


#endif /* ROBOTIS_FRAMEWORK_COMMON_INCLUDE_ROBOTIS_FRAMEWORK_COMMON_SINGLETON_H_ */
