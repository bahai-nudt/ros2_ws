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



    std::mutex _heading_mtx;
    std::mutex _imu_mtx;

private:
    // 私有构造函数
    SingletonDataBuffer(){
        _heading_buffer._max_size = 100;
        _imu_buffer._max_size = 1000;
    };

    // 禁用拷贝构造和赋值操作
    SingletonDataBuffer(const SingletonDataBuffer&) = delete;
    SingletonDataBuffer& operator=(const SingletonDataBuffer&) = delete;
};






#endif