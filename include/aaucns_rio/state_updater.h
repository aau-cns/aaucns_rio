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

#ifndef _STATE_UPDATER_H_
#define _STATE_UPDATER_H_

#include <Eigen/Dense>

#include "aaucns_rio/config.h"
#include "aaucns_rio/features.h"
#include "aaucns_rio/parameters.h"
#include "aaucns_rio/state.h"

namespace aaucns_rio
{
class StateUpdater
{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    StateUpdater() = default;
    // Return the state after correction to be published from RIO object.
    bool applyAllMeasurements(Features& features,
                              const Eigen::MatrixXd& velocities_and_points,
                              const Parameters& parameters,
                              State& closest_to_measurement_state);
    static void augmentStateAndCovarianceMatrix(State& state);

   private:
    // Called from applyMeasurement().
    void applyCorrection(const Eigen::MatrixXd& correction,
                         State& closest_to_measurement_state);
    // Compute residual, H and R matrices to be used in
    // applyMeasurement().
    void prepareMatricesForUpdate(const Parameters& parameters,
                                  const State& closest_to_measurement_state,
                                  Features& features, Eigen::MatrixXd& H,
                                  Eigen::MatrixXd& R, Eigen::MatrixXd& r);
    void prepareMatricesForPersistentFeaturesUpdate(
        const Parameters& parameters, const State& closest_to_measurement_state,
        Eigen::MatrixXd& H_pf, Eigen::MatrixXd& R_pf, Eigen::MatrixXd& r_pf);
    void prepareMatricesForVelocityUpdate(
        const Parameters& parameters, const State& closest_to_measurement_state,
        const Eigen::MatrixXd& velocities_and_points, Eigen::MatrixXd& H,
        Eigen::MatrixXd& R, Eigen::MatrixXd& r);

    // Construct one block of the H matrix which corresponds to one feature.
    // Such matrices will be stacked together to form H.
    Eigen::MatrixXd getJacobianForSingleFeature(
        const Parameters& parameters, const State& closest_to_measurement_state,
        const Eigen::VectorXd& single_matched_feature, const int index);
    Eigen::MatrixXd getPersistentFeatureJacobianForSingleFeature(
        const Parameters& parameters, const State& closest_to_measurement_state,
        const int index);
    Eigen::MatrixXd getVelocityJacobianForSingleFeature(
        const Parameters& parameters, const State& closest_to_measurement_state,
        const Eigen::VectorXd& single_velocity_and_point);
    double evaluateVelocityMeasurementEquation(
        const Parameters& parameters, const State& closest_to_measurement_state,
        const Eigen::VectorXd& single_velocity_and_point);
    Eigen::Vector3d evaluatePersistentFeatureMeasurementEquation(
        const Parameters& parameters, const State& closest_to_measurement_state,
        const int index);
};

}  // namespace aaucns_rio

#endif /* _STATE_UPDATER_H_ */
