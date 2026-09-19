//
// Created by xiang on 25-3-12.
//

#ifndef LIGHTNING_LOOP_CANDIDATE_H
#define LIGHTNING_LOOP_CANDIDATE_H

#include "common/eigen_types.h"

namespace lightning {

/**
 * 回环检测候选帧
 */
struct LoopCandidate {
    LoopCandidate() {}
    LoopCandidate(uint64_t id1, uint64_t id2) : idx1_(id1), idx2_(id2) {}

    uint64_t idx1_ = 0;
    uint64_t idx2_ = 0;
    SE3 Tij_;

    double ndt_score_ = 0.0;
    double corr_trans_ = 0.0;    // 回环 NDT 结果相对当前位姿的修正量（平移，m）
    double corr_rot_deg_ = 0.0;  // 同上（旋转，度）
};

}  // namespace lightning

#endif  // LIGHTNING_LOOP_CANDIDATE_H
