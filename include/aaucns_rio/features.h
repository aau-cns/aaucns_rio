// Copyright (C) 2024 Jan Michalczyk, Control of Networked Systems, University
// of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <jan.michalczyk@aau.at>

#ifndef _FEATURES_H_
#define _FEATURES_H_

#include <Eigen/Dense>
#include <array>
#include <iostream>

#include "aaucns_rio/config.h"

namespace aaucns_rio
{
struct MatrixAndFlag
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::MatrixXd positions;
    // TODO(jan): change to `has_this_past_pose_matchings_`.
    bool true_or_false;
};

struct MatchingResult
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr std::size_t kNumberOfMatchedCoordsAndRowColInt = 9;
    static constexpr std::size_t kNumberOfUnmatchedCoordsAndRowCol = 5;
    Eigen::Matrix<double, Eigen::Dynamic, kNumberOfMatchedCoordsAndRowColInt>
        matched_positions;
    Eigen::Matrix<double, Eigen::Dynamic, kNumberOfUnmatchedCoordsAndRowCol>
        unmatched_positions;
    bool true_or_false;
};

class Features
{
   public:
    Features() { reset(); }
    void reset()
    {
        for (auto& item : positions_and_matched_or_not_)
        {
            item.true_or_false = false;
            item.positions.resize(0, 6);
        }
    }

    bool isEmpty() const
    {
        bool is_empty = true;
        for (const auto& item : positions_and_matched_or_not_)
        {
            if (item.true_or_false)
            {
                is_empty = false;
                break;
            }
        }
        return is_empty;
    }

    std::array<MatrixAndFlag, Config::kMaxPastElements>
        positions_and_matched_or_not_;
};

inline std::ostream& operator<<(std::ostream& os, const Features& features)
{
    os << "features start" << std::endl;
    for (const auto& item : features.positions_and_matched_or_not_)
    {
        os << std::boolalpha << item.true_or_false << std::endl;
        os << item.positions << std::endl;
    }
    os << "features end" << std::endl;
    return os;
}

}  // namespace aaucns_rio

#endif /* _FEATURES_H_ */
