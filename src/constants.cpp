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

#include "aaucns_rio/constants.h"

namespace aaucns_rio
{
const Eigen::Vector3d& Constants::getGravityVector()
{
    static Eigen::Vector3d g = (Eigen::Vector3d() << 0, 0, 9.81).finished();
    return g;
}

}  // namespace aaucns_rio