#ifndef GLOC_DATA_BUFFER_H
#define GLOC_DATA_BUFFER_H

#include <iostream>
#include <string>
#include <mutex>

#include "base_utils/bounded_deque.h"
#include "base_utils/data_message.h"

class SingletonDataBuffer {

public:
    static SingletonDataBuffer& getInstance() {
        static SingletonDataBuffer instance;
        return instance;
    }

    BoundedDeque<Ins> _ins_buffer;

private:
    SingletonDataBuffer(){
        _ins_buffer._max_size = 100;
    };

    SingletonDataBuffer(const SingletonDataBuffer&) = delete;
    SingletonDataBuffer& operator=(const SingletonDataBuffer&) = delete;
};






#endif