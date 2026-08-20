#pragma once

#include <array>
#include <cstddef>

namespace msf {

/** 误差状态块 ID（顺序与 Enable / FromStateDim 一致） */
enum class BlockId : int {
    Rotation = 0,
    Velocity,
    Position,
    GyroBias,
    AccBias,
    LeverArm,
    Count  ///< 哨兵：块种类数，不是物理状态块
};

/**
 * @brief 误差向量分块表：BlockId → 在 _Xk 中的起始下标 / 是否启用
 */
class StateLayout {
public:
    void Enable(BlockId id, int dim);
    int Offset(BlockId id) const;
    bool Has(BlockId id) const;
    int Dim() const { return dim_; }

    /** 15: φ,v,p,eb,db；>15（如 18）: 再加杆臂 */
    static StateLayout FromStateDim(int state_dim);

private:
    std::array<int, static_cast<std::size_t>(BlockId::Count)> offsets_{};
    std::array<int, static_cast<std::size_t>(BlockId::Count)> dims_{};
    int dim_ = 0;
};

}  // namespace msf
