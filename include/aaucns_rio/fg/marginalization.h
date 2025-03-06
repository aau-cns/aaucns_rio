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

#ifndef _MARGINALIZATION_H_
#define _MARGINALIZATION_H_

#include <gtsam/linear/HessianFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "aaucns_rio/circular_buffer.h"
#include "aaucns_rio/config.h"
#include "aaucns_rio/fg/factors_data.h"

namespace aaucns_rio
{
class Marginalization
{
   public:
    // Returns a factor graph where the specified keys are marginalized.
    // connectedKeyCallback (can also be nullptr) will be called as soon as the
    // connected variables are computed (and before the actual marginalization
    // is performed).
    gtsam::NonlinearFactorGraph marginalizeOut(
        const gtsam::NonlinearFactorGraph& graph,
        const gtsam::Values& values_after_shift, const gtsam::Values& solution,
        const gtsam::FastVector<gtsam::Key>& keysToMarginalize);

    // Fills newGraph with factors which are not connected and
    // marginalizedOutGraph with all factors which will be marginalized out,
    // Also fills setOfKeysToMarginalize, and connectedKeys.
    void extractKeysToMarginalize(
        const gtsam::NonlinearFactorGraph& graph,
        gtsam::NonlinearFactorGraph& marginalizedOutGraph,
        gtsam::FastSet<gtsam::Key>& setOfKeysToMarginalize,
        gtsam::FastSet<gtsam::Key>& connectedKeys);

    // Compute the Schur complement with the given dimension of marginalized
    // factors and other factors.
    gtsam::Matrix computeSchurComplement(const gtsam::Matrix& augmentedHessian,
                                         int mSize, int aSize);

    gtsam::Values buildValuesForLinearization(
        const gtsam::Values& values_after_shift, const gtsam::Values& solution,
        const gtsam::FastVector<gtsam::Key>& keys_to_marginalize) const;

    // Get PF keys which dissapear from the horizon - they are only connected to
    // the pose being marginzalied out. They will also be marginalized out.
    gtsam::FastVector<gtsam::Key> getPFKeysToMarginalize(
        const CircularBuffer<RadarPFDistanceFactorData,
                             ConfigFg::kNPosesInWindow>&
            pf_factor_data_horizon);

    bool isPFKeyInPrior(const gtsam::Key key) const;

   private:
    std::pair<gtsam::Matrix, gtsam::Vector> pairFromAugmentedHessian(
        const gtsam::Matrix& matrix);
    gtsam::FastVector<gtsam::Key> sortKeys(
        const gtsam::FastSet<gtsam::Key>& set_to_sort);
    gtsam::FastVector<gtsam::Key> decrementNavStatesKeys(
        const gtsam::FastVector<gtsam::Key>& keys);
    gtsam::FastVector<gtsam::Key> getPFKeysInPrior() const;

    gtsam::FastVector<gtsam::Key> sorted_connected_keys_;
};

}  // namespace aaucns_rio

#endif  // _MARGINALIZATION_H_