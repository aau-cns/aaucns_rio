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

#ifndef _STATE_PREDICTOR_H_
#define _STATE_PREDICTOR_H_

#include <Eigen/Dense>

#include "aaucns_rio/parameters.h"
#include "aaucns_rio/state.h"

namespace aaucns_rio
{
class StatePredictor
{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    StatePredictor();
    const Eigen::Vector3d& getGravityVector() const;
    void predictState(const double dt, const State& previous_state,
                      State& current_state);
    void predictProcessCovariance(const double dt, const State& previous_state,
                                  const Parameters& parameters,
                                  State& current_state);

    // Get omega matrix (JPL notation).
    Eigen::Matrix<double, 4, 4> getOmegaJPL(
        const Eigen::Vector3d& angular_vel) const;

   private:
    Eigen::Vector3d g_;
    // discrete state propagation matrix
    Eigen::MatrixXd Fd_;
    // discrete propagation noise matrix
    Eigen::MatrixXd Qd_;
};

}  // namespace aaucns_rio

#endif /* _STATE_PREDICTOR_H_ */
