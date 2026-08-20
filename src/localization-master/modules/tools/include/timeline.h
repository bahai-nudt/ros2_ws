#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

namespace msf {
namespace tools {

/// 时间线事件：tag 由组合层约定（对应某条传感器流），index 为该流内下标。
struct Event {
    double timestamp = 0.0;
    int    tag = 0;
    size_t index = 0;
};

// 要求 T 有 _timestamp 成员。
template <typename T>
void AppendEvents(std::vector<Event>& events, const std::vector<T>& stream, int tag) {
    events.reserve(events.size() + stream.size());
    for (size_t i = 0; i < stream.size(); ++i) {
        events.push_back({stream[i]._timestamp, tag, i});
    }
}

inline void SortEvents(std::vector<Event>& events) {
    std::sort(events.begin(), events.end(),
              [](const Event& a, const Event& b) { return a.timestamp < b.timestamp; });
}

// 跳过 timestamp <= t 的事件（通常为初始化时刻及之前）。
inline size_t FirstEventAfter(const std::vector<Event>& events, double t) {
    size_t i = 0;
    while (i < events.size() && events[i].timestamp <= t) {
        ++i;
    }
    return i;
}

}  // namespace tools
}  // namespace msf
