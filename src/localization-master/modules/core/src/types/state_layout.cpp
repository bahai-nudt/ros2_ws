#include "types/state_layout.h"

namespace msf {

void StateLayout::Enable(BlockId id, int dim) {
    const auto idx = static_cast<std::size_t>(id);
    offsets_[idx] = dim_;
    dims_[idx] = dim;
    dim_ += dim;
}

int StateLayout::Offset(BlockId id) const {
    const auto idx = static_cast<std::size_t>(id);
    if (dims_[idx] <= 0) {
        return -1;
    }
    return offsets_[idx];
}

bool StateLayout::Has(BlockId id) const { return Offset(id) >= 0; }

StateLayout StateLayout::FromStateDim(int state_dim) {
    StateLayout layout;
    layout.Enable(BlockId::Rotation, 3);
    layout.Enable(BlockId::Velocity, 3);
    layout.Enable(BlockId::Position, 3);
    layout.Enable(BlockId::GyroBias, 3);
    layout.Enable(BlockId::AccBias, 3);
    if (state_dim > 15) {
        layout.Enable(BlockId::LeverArm, 3);
    }
    return layout;
}

}  // namespace msf
