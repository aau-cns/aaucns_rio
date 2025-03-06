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

#ifndef _TRAILPOINT_H_
#define _TRAILPOINT_H_

#include <Eigen/Dense>

#include "aaucns_rio/circular_buffer.h"
#include "aaucns_rio/config.h"

namespace aaucns_rio
{
struct TrailPoint
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // First three entries are coords of previous point and last three of
    // current. In case of a PF, .matches_history[0].head(3) holds the latest
    // match of a PF.
    static constexpr std::size_t kNumberOfMatchedCoords = 6;
    using HistoryElementType = Eigen::Matrix<double, 1, kNumberOfMatchedCoords>;
    CircularBuffer<HistoryElementType, Config::kMaxPastElements>
        matches_history;
    // Coordinates of the point from the latest pointcloud not yet matched.
    static constexpr std::size_t kNumberOfCoords = 3;
    using CoordType = Eigen::Matrix<double, 1, kNumberOfCoords>;
    CoordType most_recent_coordinates;
    // Is the feature active and should be used in the current update.
    bool is_active;
    double intensity;
    bool is_persistent;
    // Id used only in FG part now and only set for PFs..
    std::size_t id;
};

inline std::ostream& operator<<(std::ostream& os, const TrailPoint& trailpoint)
{
    os << "trailpoint start" << std::endl;
    os << "is active: " << std::boolalpha << trailpoint.is_active << std::endl;
    os << "is persistent: " << std::boolalpha << trailpoint.is_persistent
       << std::endl;
    os << "matches history: " << std::endl
       << trailpoint.matches_history << std::endl;
    os << "most recent coords: " << trailpoint.most_recent_coordinates
       << std::endl;
    os << "intensity: " << trailpoint.intensity << std::endl;
    os << "id: " << trailpoint.id << std::endl;
    os << "trailpoint end" << std::endl;
    return os;
}

}  // namespace aaucns_rio

#endif /* _TRAILPOINT_H_ */
