#pragma once

#include "config/global_config.h"
#include "interfaces/meas_factor.h"
#include "math/earth.h"
#include "types/nav_state.h"
#include "types/state_layout.h"

namespace msf {
namespace algorithms {

/**
 * 零速检测（ZUPT 判定）：结合外部速度与 INS 名义状态判定静止并累计时长。
 * 只负责“判静止”；零速量测由 ZeroVelocityFactor 构建（功能依附骨架，而非反向）。
 */
class ZuptDetector {
public:
    explicit ZuptDetector(const GlobalConfig::QualityControl::Zupt& zupt);

    /**
     * 每帧速度量测调用：更新静止判定与累计时长。
     * @return 当前是否判定静止（静止条件持续 >= min_duration_sec）。
     */
    bool Update(double timestamp, double hor_speed, double ver_speed,
                const NominalState& nominal);

    bool Active() const { return active_; }
    double StaticDuration() const { return static_duration_; }
    void Reset();

private:
    GlobalConfig::QualityControl::Zupt zupt_;

    double static_duration_ = 0.0;
    bool has_prev_ts_ = false;
    double prev_ts_ = 0.0;
    double last_timestamp_ = 0.0;
    bool active_ = false;
};

/**
 * 零速速度量测因子：z = vn_ins_L（量测为 0），H 与速度量测一致，R = ZUPT 噪声。
 * 仅在 ZuptDetector 判定静止后由组合层调用。
 */
class ZeroVelocityFactor : public MeasFactor {
public:
    explicit ZeroVelocityFactor(const GlobalConfig::QualityControl::Zupt& zupt);

    MeasBlock BuildMeasBlock(const NominalState& nominal, earth& eth,
                             const StateLayout& layout, const IterationContext& ctx) override;

private:
    GlobalConfig::QualityControl::Zupt zupt_;
};

}  // namespace algorithms
}  // namespace msf
