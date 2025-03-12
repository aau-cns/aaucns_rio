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

#include "aaucns_rio/fg/rio_fg.h"

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/Preconditioner.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>

#include <cmath>
#include <limits>
#include <memory>

#include "aaucns_rio/constants.h"
#include "aaucns_rio/debug.h"
#include "aaucns_rio/fg/filter_state_factory.h"
#include "aaucns_rio/fg/imu_measurement.h"
#include "aaucns_rio/fg/marginalization.h"
#include "aaucns_rio/fg/pf_distance_factor.h"
#include "aaucns_rio/fg/radar_distance_factor.h"
#include "aaucns_rio/fg/radar_velocity_factor.h"
#include "aaucns_rio/velocity_provider.h"
#include "gtsam/linear/LossFunctions.h"
#include "yaml-cpp/yaml.h"

namespace aaucns_rio
{
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::P;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

// TODO(jan): Replace `dump_features` with `Config::kWriteFeaturesToFile`.
RIOFg::RIOFg(const std::string& pc2_topic, const std::string& imu_topic,
             const std::string& pose_topic, const std::string& state_topic,
             const std::string& config_filename, const bool dump_features,
             ros::NodeHandle& nh)
    : points_associator_(dump_features),
      trail_(),
      logger_(Config::kEnableLogging),
      config_filename_(config_filename),
      pc2_topic_(pc2_topic),
      imu_topic_(imu_topic),
      nh_(nh)
{
    state_pub_ = nh.advertise<aaucns_rio::DoubleArrayStamped>(state_topic, 1);
    pose_pub_ =
        nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic, 1);
    pose_at_update_pub_ =
        nh.advertise<geometry_msgs::PoseStamped>("/pose_at_update", 1);
    initialization_service_ =
        nh.advertiseService("init_service", &RIOFg::initServiceCallback, this);
}

bool RIOFg::initServiceCallback(std_srvs::SetBool::Request& /*request*/,
                                std_srvs::SetBool::Response& res)
{
    initialize(config_filename_);
    setupSubscribers(pc2_topic_, imu_topic_, nh_);
    res.success = true;
    ROS_INFO_STREAM("Initialized filter trough ROS Service.");
    return true;
}

RIOFg::~RIOFg(){};

void RIOFg::setupSubscribers(const std::string& pc2_topic,
                             const std::string& imu_topic, ros::NodeHandle& nh)
{
    pc2_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
        pc2_topic, 1, &RIOFg::PC2Callback, this);
    imu_sub_ =
        nh.subscribe<sensor_msgs::Imu>(imu_topic, 1, &RIOFg::IMUCallback, this);
}

void RIOFg::PC2Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    if (!is_initialized_)
    {
        return;
    }

    pcl::PointCloud<RadarPointCloudType> current_pc2;
    if (!rosPCLMsgToRadarPCL(*msg, current_pc2))
    {
        return;
    }

    const double current_pc2_timestamp_s = msg->header.stamp.toSec();
    if (hasPC2CallbackBeenCalledEnoughTimes())
    {
        // Build data to construct graph and values from.
        std::shared_ptr<gtsam::PreintegratedCombinedMeasurements>
            imu_integrator = preintegrateIMUMeasurements(
                previous_pc2_timestamp_s_, current_pc2_timestamp_s);

        // Predict the new state based on the pre-integration.
        // Predicted state is the state at which the current measurement
        // was taken. `initial_state_` is the most recent updated estimate.
        gtsam::NavState initial_state = initial_state_;
        initial_state =
            imu_integrator->predict(initial_state, initial_imu_bias_);

        // Get velocities and create a radial velocity factor.
        const Eigen::MatrixXd velocities_and_points =
            VelocityProvider::getPointsAndVelocities(current_pc2, logger_);

        int current_pc2_timestamp_s_index;
        const IMUMeasurement& imu_measurement = getClosestIMUMeasurement(
            current_pc2_timestamp_s, current_pc2_timestamp_s_index, 0.0);

        gtsam::noiseModel::Diagonal::shared_ptr radial_velocity_factor_noise =
            gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(1)
                 << parameters_.noise_meas3_ * parameters_.noise_meas3_)
                    .finished());
        gtsam::noiseModel::mEstimator::DCS::shared_ptr cauchy_loss(
            new gtsam::noiseModel::mEstimator::DCS(4.0));
        gtsam::noiseModel::Robust::shared_ptr velocity_robust_noise(
            new gtsam::noiseModel::Robust(cauchy_loss,
                                          radial_velocity_factor_noise));

        // ----------------Obtain data for building factors----------------
        // Build and push velocity factor data.
        velocity_factor_data_horizon_.push_back(RadialVelocityFactorData(
            velocities_and_points, imu_measurement.w_m_));

        // Build and push IMU factor data.
        imu_factor_data_horizon_.push_back(IMUFactorData(imu_integrator));

        if (trail_.isInitialized())
        {
            // Convert gtsam data into State class to get the current state.
            FilterStateFactory::updateState(initial_state, initial_state_,
                                            imu_measurement, initial_imu_bias_,
                                            radar_imu_transform_,
                                            most_recent_updated_state_);
            // Below in `calculate()` we append persistent features into the
            // points for matching the new scan with.
            if (!points_associator_.calculateFeaturesFromTrail(
                    current_pc2, most_recent_updated_state_, parameters_,
                    trail_, features_))
            {
                std::cerr << "No features found." << std::endl;
            }
            else
            {
                // Look for persistent features in the trail.
                if (Config::kEnablePersistentFeatures)
                {
                    trail_.findAndAddPersistentFeaturesToState(
                        most_recent_updated_state_, parameters_);
                }
            }
        }
        // Build and push distance factor data.
        // `features_` is empty if no matches found.
        distance_factor_data_horizon_.push_back(
            RadarDistanceFactorData(features_));

        // Build and push distance factor data.

        pf_factor_data_horizon_.push_back(
            RadarPFDistanceFactorData(most_recent_updated_state_));

        // ----------------Create factors from the built data----------------
        std::vector<RadarDistanceFactor> distance_factors =
            RadarDistanceFactorFactory::createFactors(
                distance_factor_data_horizon_, parameters_,
                radar_imu_transform_, trail_, poses_in_window_);
        for (const RadarDistanceFactor& factor : distance_factors)
        {
            graph_.add(factor);
        }

        initializePFsValues();
        std::vector<RadarPFDistanceFactor> radar_pf_distance_factors =
            RadarPFDistanceFactorFactory::createFactors(
                pf_factor_data_horizon_, parameters_, radar_imu_transform_);
        for (const RadarPFDistanceFactor& factor : radar_pf_distance_factors)
        {
            graph_.add(factor);
        }

        for (int i = 0; i < imu_factor_data_horizon_.size(); ++i)
        {
            const gtsam::CombinedImuFactor imu_factor(
                X(i), V(i), X(i + 1), V(i + 1), B(i), B(i + 1),
                *imu_factor_data_horizon_[i].imu_integrator_);
            graph_.add(imu_factor);
        }

        for (int i = 0; i < velocity_factor_data_horizon_.size(); ++i)
        {
            for (int j = 0;
                 j <
                 velocity_factor_data_horizon_[i].velocities_and_points_.rows();
                 ++j)
            {
                const RadialVelocityFactor radial_velocity_factor(
                    X(i), V(i), B(i),
                    velocity_factor_data_horizon_[i].velocities_and_points_.row(
                        j)(0),
                    velocity_factor_data_horizon_[i]
                        .velocities_and_points_.row(j)
                        .tail(3),
                    radar_imu_transform_, velocity_factor_data_horizon_[i].w_m_,
                    velocity_robust_noise);
                graph_.add(radial_velocity_factor);
            }
        }

        // Initialize values for the current pose.
        values_.insert(X(poses_in_window_), initial_state.pose());
        values_.insert(V(poses_in_window_), initial_state.v());
        values_.insert(B(poses_in_window_), initial_imu_bias_);

        // ----------------Solve----------------
        // graph_.print("\nFactor Graph:\n");
        gtsam::LevenbergMarquardtParams params =
            gtsam::LevenbergMarquardtParams::CeresDefaults();
        // params.verbosityLM = gtsam::LevenbergMarquardtParams::SUMMARY;
        gtsam::LevenbergMarquardtOptimizer optimizer_lm(graph_, values_,
                                                        params);
        const gtsam::Values solution = optimizer_lm.optimize();

        initial_pose_ = solution.at<gtsam::Pose3>(X(poses_in_window_));
        initial_velocity_ = solution.at<gtsam::Vector3>(V(poses_in_window_));
        initial_state_ = gtsam::NavState(initial_pose_, initial_velocity_);
        initial_imu_bias_ =
            solution.at<gtsam::imuBias::ConstantBias>(B(poses_in_window_));
        previous_solution_ = solution;

        if (poses_in_window_ < ConfigFg::kNPosesInWindow)
        {
            graph_.resize(0);
        }
        values_.clear();

        initializeSolution();

        outputPose(current_pc2_timestamp_s, msg->header.seq);

        releasePFsIds();
    }
    else
    {
        trail_.initialize(current_pc2);
        increasePC2CallbackTimesCalledCounter();
    }
    previous_pc2_timestamp_s_ = current_pc2_timestamp_s;
}

void RIOFg::marginalize()
{
    // Get PFs to marginalize out.
    const gtsam::FastVector<gtsam::Key> pfs_keys_to_marginalize_out =
        marginalization_.getPFKeysToMarginalize(pf_factor_data_horizon_);
    gtsam::FastVector<gtsam::Key> to_marginalize_out{B(0), V(0), X(0)};
    to_marginalize_out.insert(to_marginalize_out.end(),
                              pfs_keys_to_marginalize_out.begin(),
                              pfs_keys_to_marginalize_out.end());
    graph_ = marginalization_.marginalizeOut(
        graph_, values_, previous_solution_, to_marginalize_out);
}

void RIOFg::initializeSolution()
{
    if (poses_in_window_ == ConfigFg::kNPosesInWindow)
    {
        for (int i = 0; i < poses_in_window_; ++i)
        {
            values_.insert(X(i), previous_solution_.at<gtsam::Pose3>(X(i + 1)));
            values_.insert(V(i),
                           previous_solution_.at<gtsam::Vector3>(V(i + 1)));
            values_.insert(
                B(i),
                previous_solution_.at<gtsam::imuBias::ConstantBias>(B(i + 1)));
        }
        marginalize();
    }
    else
    {
        if (!previous_solution_.empty())
        {
            gtsam::PriorFactor<gtsam::Pose3> prior_pose(
                X(0), previous_solution_.at<gtsam::Pose3>(X(0)),
                prior_pose_noise_);
            graph_.add(prior_pose);

            gtsam::PriorFactor<gtsam::Vector3> prior_velocity(
                V(0), previous_solution_.at<gtsam::Vector3>(V(0)),
                prior_velocity_noise_);
            graph_.add(prior_velocity);

            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> prior_bias(
                B(0), previous_solution_.at<gtsam::imuBias::ConstantBias>(B(0)),
                prior_bias_noise_);
            graph_.add(prior_bias);

            for (int i = 0; i < poses_in_window_ + 1; ++i)
            {
                values_.insert(X(i), previous_solution_.at<gtsam::Pose3>(X(i)));
                values_.insert(V(i),
                               previous_solution_.at<gtsam::Vector3>(V(i)));
                values_.insert(
                    B(i),
                    previous_solution_.at<gtsam::imuBias::ConstantBias>(B(i)));
            }
        }
    }
    if (poses_in_window_ < ConfigFg::kNPosesInWindow)
    {
        ++poses_in_window_;
    }
}

void RIOFg::initializePFsValues()
{
    for (int j = 0; j < pf_factor_data_horizon_.size(); ++j)
    {
        for (int i = 0;
             i < pf_factor_data_horizon_[j]
                     .most_recent_updated_state_.persistent_features_.size();
             ++i)
        {
            // PF already existed in the previous run.
            if (previous_solution_.exists(
                    P(pf_factor_data_horizon_[j]
                          .most_recent_updated_state_.persistent_features_[i]
                          .id)) &&
                !values_.exists(
                    P(pf_factor_data_horizon_[j]
                          .most_recent_updated_state_.persistent_features_[i]
                          .id)))
            {
                values_.insert(
                    P(pf_factor_data_horizon_[j]
                          .most_recent_updated_state_.persistent_features_[i]
                          .id),
                    previous_solution_.at<gtsam::Point3>(P(
                        pf_factor_data_horizon_[j]
                            .most_recent_updated_state_.persistent_features_[i]
                            .id)));
                // Here check if it has not been included in the marginalized
                // prior. That is, if it had been connected to the x0 and also
                // other poses.
                if (!marginalization_.isPFKeyInPrior(P(
                        pf_factor_data_horizon_[j]
                            .most_recent_updated_state_.persistent_features_[i]
                            .id)))
                {
                    gtsam::PriorFactor<gtsam::Point3> prior_pf_factor(
                        P(pf_factor_data_horizon_[j]
                              .most_recent_updated_state_
                              .persistent_features_[i]
                              .id),
                        previous_solution_.at<gtsam::Point3>(
                            P(pf_factor_data_horizon_[j]
                                  .most_recent_updated_state_
                                  .persistent_features_[i]
                                  .id)),
                        prior_pf_noise_);
                    graph_.add(prior_pf_factor);
                }
                trail_.initializeId(
                    pf_factor_data_horizon_[j]
                        .most_recent_updated_state_.persistent_features_[i]
                        .id);
            }
            else if (!values_.exists(P(
                         pf_factor_data_horizon_[j]
                             .most_recent_updated_state_.persistent_features_[i]
                             .id)))
            {
                // PF does not exist so initialize it.
                values_.insert(
                    P(pf_factor_data_horizon_[j]
                          .most_recent_updated_state_.persistent_features_[i]
                          .id),
                    gtsam::Point3(
                        pf_factor_data_horizon_[j]
                            .most_recent_updated_state_.persistent_features_[i]
                            .most_recent_coordinates.transpose()));

                gtsam::PriorFactor<gtsam::Point3> prior_pf_factor(
                    P(pf_factor_data_horizon_[j]
                          .most_recent_updated_state_.persistent_features_[i]
                          .id),
                    gtsam::Point3(
                        pf_factor_data_horizon_[j]
                            .most_recent_updated_state_.persistent_features_[i]
                            .most_recent_coordinates.transpose()),
                    prior_pf_noise_);

                graph_.add(prior_pf_factor);

                trail_.initializeId(
                    pf_factor_data_horizon_[j]
                        .most_recent_updated_state_.persistent_features_[i]
                        .id);
            }
            else
            {
            }
        }
    }
}

void RIOFg::releasePFsIds()
{
    std::vector<std::size_t> all_initialized_ids =
        trail_.getAllInitializedIds();
    for (int k = 0; k < all_initialized_ids.size(); ++k)
    {
        bool not_found = true;
        for (int j = 0; j < pf_factor_data_horizon_.size(); ++j)
        {
            for (int i = 0;
                 i <
                 pf_factor_data_horizon_[j]
                     .most_recent_updated_state_.persistent_features_.size();
                 ++i)
            {
                if (all_initialized_ids[k] ==
                    pf_factor_data_horizon_[j]
                        .most_recent_updated_state_.persistent_features_[i]
                        .id)
                {
                    not_found = false;
                }
            }
        }
        if (not_found)
        {
            trail_.resetId(all_initialized_ids[k]);
        }
    }
}

void RIOFg::IMUCallback(const sensor_msgs::ImuConstPtr& msg)
{
    IMUMeasurement imu_measurement(msg);
    imu_measurements_.push_back(imu_measurement);
}

std::shared_ptr<gtsam::PreintegratedCombinedMeasurements>
RIOFg::preintegrateIMUMeasurements(const double ti_s, const double tf_s)
{
    // Calculate indexes of the measurements.
    int index_ti_s;
    (void)getClosestIMUMeasurement(ti_s, index_ti_s, 0.0);
    int index_tf_s;
    (void)getClosestIMUMeasurement(tf_s, index_tf_s, 0.0);

    // TODO(jan): Throw an exception if no elements to preintegrate.
    std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imu_integrator =
        std::make_shared<gtsam::PreintegratedCombinedMeasurements>(
            preintegration_params_, initial_imu_bias_);
    if (imu_measurements_.size() > 0)
    {
        imu_integrator->resetIntegrationAndSetBias(initial_imu_bias_);
        for (int i = index_ti_s; i < index_tf_s; ++i)
        {
            const double dt = imu_measurements_[i + 1].timestamp_s_ -
                              imu_measurements_[i].timestamp_s_;
            imu_integrator->integrateMeasurement(
                gtsam::Vector3(imu_measurements_[i].a_m_(0),
                               imu_measurements_[i].a_m_(1),
                               imu_measurements_[i].a_m_(2)),
                gtsam::Vector3(imu_measurements_[i].w_m_(0),
                               imu_measurements_[i].w_m_(1),
                               imu_measurements_[i].w_m_(2)),
                dt);
        }
    }
    return imu_integrator;
}

void RIOFg::outputPose(const double ti_f, const uint32_t seq)
{
    geometry_msgs::PoseWithCovarianceStampedPtr msg_pose(
        new geometry_msgs::PoseWithCovarianceStamped);
    msg_pose->header.stamp = ros::Time(ti_f);
    msg_pose->header.seq = seq;
    util::vector3dToPoint(initial_state_.t(), msg_pose->pose.pose.position);
    util::gtsamQuaternionToMsg(initial_state_.quaternion(),
                               msg_pose->pose.pose.orientation);
    writePose(msg_pose);
}

IMUMeasurement& RIOFg::getClosestIMUMeasurement(const double timestamp_s,
                                                int& index_to_return,
                                                const double delay)
{
    int index = imu_measurements_.size() - 1U;
    double time_distance = std::numeric_limits<double>::max();
    const double time_now = timestamp_s - delay;
    while (std::abs(time_now - imu_measurements_[index].timestamp_s_) <
           time_distance)
    {
        // time_distance decreases continuously until best point
        // reached.
        time_distance =
            std::abs(time_now - imu_measurements_[index].timestamp_s_);
        --index;
    }
    // We subtracted one too many before.
    ++index;
    index_to_return = index;
    return imu_measurements_[index];
}

void RIOFg::writeState(const aaucns_rio::DoubleArrayStampedPtr& msg)
{
    state_pub_.publish(msg);
};

void RIOFg::writePose(const geometry_msgs::PoseWithCovarianceStampedPtr& msg)
{
    pose_pub_.publish(msg);
};

void RIOFg::initialize(const std::string& config_file)
{
    initializeStateFromConfig(config_file);
    initializeParameters();
    // Mark the end of initialization.
    is_initialized_ = true;
}

void RIOFg::initializeParameters()
{
    gtsam::PriorFactor<gtsam::Pose3> prior_pose(X(0), initial_pose_,
                                                prior_pose_noise_);
    graph_.add(prior_pose);

    gtsam::PriorFactor<gtsam::Vector3> prior_velocity(V(0), initial_velocity_,
                                                      prior_velocity_noise_);
    graph_.add(prior_velocity);

    gtsam::PriorFactor<gtsam::imuBias::ConstantBias> prior_bias(
        B(0), initial_imu_bias_, prior_bias_noise_);
    graph_.add(prior_bias);

    values_.insert(X(0), initial_pose_);
    values_.insert(V(0), initial_velocity_);
    values_.insert(B(0), initial_imu_bias_);

    poses_in_window_ = 1;
}

void RIOFg::initializeStateFromConfig(const std::string& config_file)
{
    const YAML::Node config = YAML::LoadFile(config_file);

    // Initial navigation state. This state gets constantly updated
    // after each prediction and serves as next initial state.
    initial_pose_ = gtsam::Pose3(
        gtsam::Rot3(config["qw"].as<double>(), config["qx"].as<double>(),
                    config["qy"].as<double>(), config["qz"].as<double>()),
        gtsam::Point3(config["px"].as<double>(), config["py"].as<double>(),
                      config["pz"].as<double>()));
    initial_velocity_ =
        gtsam::Vector3(config["vx"].as<double>(), config["vy"].as<double>(),
                       config["vz"].as<double>());
    initial_state_ = gtsam::NavState(initial_pose_, initial_velocity_);
    // Set initial biases to zero.
    initial_imu_bias_ = gtsam::imuBias::ConstantBias(
        (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());

    // Extrinsic calibration parameters.
    radar_imu_transform_ = gtsam::Pose3(
        gtsam::Rot3(config["q_riw"].as<double>(), config["q_rix"].as<double>(),
                    config["q_riy"].as<double>(), config["q_riz"].as<double>()),
        gtsam::Point3(config["p_rix"].as<double>(),
                      config["p_riy"].as<double>(),
                      config["p_riz"].as<double>()));

    most_recent_updated_state_ = FilterStateFactory::createState(
        initial_state_, initial_state_, IMUMeasurement(), initial_imu_bias_,
        radar_imu_transform_);

    // Preintegration parameters. Gravity down by default.
    preintegration_params_ = gtsam::PreintegrationCombinedParams::MakeSharedU();
    preintegration_params_->accelerometerCovariance =
        gtsam::Matrix33::Identity(3, 3) *
        std::pow(config["noise_acc"].as<double>(), 2);
    preintegration_params_->gyroscopeCovariance =
        gtsam::Matrix33::Identity(3, 3) *
        std::pow(config["noise_gyr"].as<double>(), 2);
    // TODO(jan): What is integration covariance?
    preintegration_params_->integrationCovariance =
        gtsam::Matrix33::Identity(3, 3) * std::pow(1e-4, 2);

    preintegration_params_->biasAccCovariance =
        gtsam::I_3x3 * pow(config["noise_accbias"].as<double>(), 2);
    ;  // acc bias in continuous
    preintegration_params_->biasOmegaCovariance =
        gtsam::I_3x3 * pow(config["noise_gyrbias"].as<double>(), 2);
    ;  // gyro bias in continuous
    preintegration_params_->biasAccOmegaInt = gtsam::I_6x6 * 1e-5;

    prior_pose_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 0.015, 0.015, 0.015, 0.015, 0.015, 0.015)
            .finished());  // rad,rad,rad,m, m, m
    prior_velocity_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(3) << 0.01, 0.01, 0.01).finished());  // m/s
    prior_bias_noise_ = gtsam::noiseModel::Isotropic::Sigmas(
        (gtsam::Vector(6) << 0.3, 0.3, 0.3, 0.005, 0.005, 0.005).finished());
    prior_pf_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(3) << config["noise_pf_x"].as<double>(),
         config["noise_pf_y"].as<double>(),
         config["noise_pf_z"].as<double>())
            .finished());  // m, m, m

    // Initialize noise parameters.
    parameters_.noise_meas1_ = config["noise_meas1"].as<double>();
    parameters_.noise_meas2_ = config["noise_meas2"].as<double>();
    parameters_.noise_meas3_ = config["noise_meas3"].as<double>();
    parameters_.noise_meas4_ = config["noise_meas4"].as<double>();
}

}  // namespace aaucns_rio
