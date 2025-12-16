#ifndef GLOC_DATA_BUFFER_H
#define GLOC_DATA_BUFFER_H

#include <iostream>
#include <string>
#include <mutex>

#include "base_utils/bounded_deque.h"
#include "base_utils/data_message.h"

#include "gloc/message.h"

class SingletonDataBuffer {

public:
    static SingletonDataBuffer& getInstance() {
        static SingletonDataBuffer instance;
        return instance;
    }

    BoundedDeque<ImuData> _imu_buffer;
    BoundedDeque<Heading> _heading_buffer;
    BoundedDeque<GpsData> _gps_buffer;

    std::mutex _heading_mtx;
    std::mutex _imu_mtx;

private:
    SingletonDataBuffer(){
        _heading_buffer._max_size = 100;
        _imu_buffer._max_size = 1000;
        _gps_buffer._max_size = 100;
    };

    SingletonDataBuffer(const SingletonDataBuffer&) = delete;
    SingletonDataBuffer& operator=(const SingletonDataBuffer&) = delete;
};






#endif