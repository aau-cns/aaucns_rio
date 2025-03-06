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

#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#include <Eigen/Dense>

namespace aaucns_rio
{
class Constants
{
   public:
    static const Eigen::Vector3d& getGravityVector();
};

}  // namespace aaucns_rio

#endif /* CONSTANTS_H_ */
