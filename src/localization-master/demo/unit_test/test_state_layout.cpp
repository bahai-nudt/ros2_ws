#include <iostream>

#include "ekf_state.h"

int main() {
    msf::EkfState state18(18);
    std::cout << "[EkfState(18)]\n"
              << "  Dim=" << state18.Dim() << "\n"
              << "  error._nx=" << state18._error._nx << "\n"
              << "  Xk.size=" << state18._error._Xk.size() << "\n"
              << "  Pk=" << state18._error._Pk.rows() << "x" << state18._error._Pk.cols()
              << "\n"
              << "  Velocity offset=" << state18.Offset(msf::BlockId::Velocity) << "\n"
              << "  Has LeverArm=" << std::boolalpha << state18.Has(msf::BlockId::LeverArm)
              << "\n"
              << "  nominal._vn=" << state18._nominal._vn.transpose() << "\n";

    msf::EkfState state15;
    state15.Init(15);
    std::cout << "[EkfState::Init(15)]\n"
              << "  Dim=" << state15.Dim() << "\n"
              << "  Has LeverArm=" << state15.Has(msf::BlockId::LeverArm) << "\n"
              << "  GyroBias offset=" << state15.Offset(msf::BlockId::GyroBias) << "\n";

    const msf::StateLayout layout18 = msf::StateLayout::FromStateDim(18);
    std::cout << "[StateLayout::FromStateDim(18)] LeverArm offset="
              << layout18.Offset(msf::BlockId::LeverArm) << "\n";

    return 0;
}
