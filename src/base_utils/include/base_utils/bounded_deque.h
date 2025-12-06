#ifndef BOUNDED_DEQUE_H
#define BOUNDED_DEQUE_H

#include <iostream>
#include <string>
#include <queue>


template<typename T>
class BoundedDeque {
public:
    BoundedDeque() :
        _max_size(1000),
        _cur_size(0)
    {
    }

    std::deque<T> cache;

    void push(T data) {
        if (_cur_size >= _max_size) {
            cache.pop_front();
            cache.push_back(data);
        }
        else {
            cache.push_back(data);
            _cur_size++;
        }

    }

    const T& operator[](size_t i) const { return cache.at(i); }
          T& operator[](size_t i)       { return cache.at(i); }

    size_t _cur_size;
    size_t _max_size;
};



#endif