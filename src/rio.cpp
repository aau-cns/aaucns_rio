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

#include "aaucns_rio/rio.h"

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>

#include <cmath>
#include <limits>
#include <memory>

#include "aaucns_rio/DoubleArrayStamped.h"
#include "aaucns_rio/debug.h"
#include "aaucns_rio/velocity_provider.h"
#include "yaml-cpp/yaml.h"
namespace aaucns_rio
{
// TODO(jan): Replace `dump_features` with `Config::kWriteFeaturesToFile`.
RIO::RIO(const std::string& pc2_topic, const std::string& imu_topic,
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
        nh.advertiseService("init_service", &RIO::initServiceCallback, this);
}

bool RIO::initServiceCallback(std_srvs::SetBool::Request& /*request*/,
                              std_srvs::SetBool::Response& res)
{
    initialize(config_filename_);
    setupSubscribers(pc2_topic_, imu_topic_, nh_);
    res.success = true;
    ROS_INFO_STREAM("Initialized filter trough ROS Service.");
    return true;
}

RIO::~RIO(){};

void RIO::setupSubscribers(const std::string& pc2_topic,
                           const std::string& imu_topic, ros::NodeHandle& nh)
{
    pc2_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(pc2_topic, 1,
                                                      &RIO::PC2Callback, this);
    imu_sub_ =
        nh.subscribe<sensor_msgs::Imu>(imu_topic, 1, &RIO::ImuCallback, this);
}

void RIO::PC2Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    if (!is_initialized_ || !is_prediction_made_)
    {
        return;
    }

    pcl::PointCloud<pcl::PointXYZI> current_pc2;
    pcl::fromROSMsg(*msg, current_pc2);

    // Get the closest state to the pointcloud timestamp and the corresponding
    // pointcloud.
    uint8_t closest_to_measurement_state_index;
    State& closest_to_measurement_state = getClosestState(
        msg->header.stamp, closest_to_measurement_state_index, 0.0);
    const Eigen::MatrixXd velocities_and_points =
        VelocityProvider::getPointsAndVelocities(current_pc2, logger_);
    if (trail_.isInitialized())
    {
        // Below in `calculate()` we append persistent features into the
        // points for matching the new scan with.
        if (!points_associator_.calculateFeaturesFromTrail(
                current_pc2, closest_to_measurement_state, parameters_, trail_,
                features_))
        {
            std::cerr << "No features found." << std::endl;
        }
        if (!state_updater_.applyAllMeasurements(
                features_, velocities_and_points, parameters_,
                closest_to_measurement_state))
        {
            std::cerr << "No measurement applied." << std::endl;
        }
        // Look for persistent features in the trail.
        if (Config::kEnablePersistentFeatures)
        {
            trail_.findAndAddPersistentFeaturesToState(
                closest_to_measurement_state, parameters_);
        }
        // Augment the covariance matrix.
        StateUpdater::augmentStateAndCovarianceMatrix(
            closest_to_measurement_state);
        predictToNewestState(closest_to_measurement_state_index);
        outputState(msg);
        outputPoseAtUpdate(msg);
    }
    else
    {
        // Save the current pc2 before returning and the pose at which
        // it has been taken.
        StateUpdater::augmentStateAndCovarianceMatrix(
            closest_to_measurement_state);
        predictToNewestState(closest_to_measurement_state_index);
        trail_.initialize(current_pc2);
    }
}

void RIO::predictToNewestState(const uint8_t closest_to_measurement_state_index)
{
    // Predict states from the updated time to the latest time in order
    // to propagate as well the previous radar pose and generally the
    // corrected state variables.
    const uint8_t current_filled_state_index = state_index_ - 2U;
    state_index_ = closest_to_measurement_state_index;
    while (state_index_ < current_filled_state_index)
    {
        const uint8_t next_state_index = state_index_ + 1U;
        const double dt =
            states_[next_state_index].time_ - states_[state_index_].time_;
        state_predictor_.predictState(dt, states_[state_index_],
                                      states_[next_state_index]);
        state_predictor_.predictProcessCovariance(
            dt, states_[state_index_], parameters_, states_[next_state_index]);
        shiftStateIndexForwards();
    }
    shiftStateIndexForwards();
    shiftStateIndexForwards();
}

void RIO::outputState(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    aaucns_rio::DoubleArrayStampedPtr msg_state(
        new aaucns_rio::DoubleArrayStamped);
    msg_state->header.stamp = msg->header.stamp;
    msg_state->header.seq = msg->header.seq;
    // Add one since we publish the nominal state.
    msg_state->data.resize(State::kNImuState + 1 + State::kNCalibState + 1, 0);
    // Publish the "previous" state since it's the last one
    // with the EKF-updated values (state_index_ - 2). state_index_
    // always points to "unfilled" state.
    getPreviousState().toStateMsg(msg_state);
    writeState(msg_state);
}

void RIO::ImuCallback(const sensor_msgs::ImuConstPtr& msg)
{
    if (!is_initialized_)
    {
        return;
    }

    // Do EKF prediction here and make fresh state available for
    // the update step.
    State& current_state = getCurrentState();
    // Get inputs.
    current_state.a_m_ << msg->linear_acceleration.x,
        msg->linear_acceleration.y, msg->linear_acceleration.z;
    current_state.w_m_ << msg->angular_velocity.x, msg->angular_velocity.y,
        msg->angular_velocity.z;
    current_state.time_ = msg->header.stamp.toSec();
    const State& previous_state = getPreviousState();
    const double dt = current_state.time_ - previous_state.time_;
    // We do not predict on the first imu message in order to avoid a huge
    // dt value from the initial state.
    if (hasImuCallbackBeenCalledEnoughTimes())
    {
        state_predictor_.predictState(dt, previous_state, current_state);
        // Predict error-state covariance.
        state_predictor_.predictProcessCovariance(dt, previous_state,
                                                  parameters_, current_state);
        is_prediction_made_ = true;

        // Publish integrated imu pose.
        outputPose(msg);
    }
    else
    {
        increaseImuCallbackTimesCalledCounter();
    }
    shiftStateIndexForwards();
}

void RIO::outputPose(const sensor_msgs::ImuConstPtr& msg)
{
    geometry_msgs::PoseWithCovarianceStampedPtr msg_pose(
        new geometry_msgs::PoseWithCovarianceStamped);
    msg_pose->header.stamp = msg->header.stamp;
    msg_pose->header.seq = msg->header.seq;
    getCurrentState().toPoseMsg(msg_pose);
    writePose(msg_pose);
}

void RIO::outputPoseAtUpdate(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    geometry_msgs::PoseStampedPtr msg_pose(new geometry_msgs::PoseStamped);
    msg_pose->header.stamp = msg->header.stamp;
    msg_pose->header.seq = msg->header.seq;
    getPreviousState().toPoseNoCovMsg(msg_pose);
    pose_at_update_pub_.publish(msg_pose);
}

void RIO::writeState(const aaucns_rio::DoubleArrayStampedPtr& msg)
{
    state_pub_.publish(msg);
};

void RIO::writePose(const geometry_msgs::PoseWithCovarianceStampedPtr& msg)
{
    pose_pub_.publish(msg);
};

void RIO::initialize(const std::string& config_file)
{
    for (auto& state : states_)
    {
        state.reset();
    }
    // Initialize the initial state and covariance and increase the state index.
    initializeStateFromConfig(config_file);
    initializeCovariance();
    shiftStateIndexForwards();
    // Mark the end of initialization.
    is_initialized_ = true;
}

void RIO::initializeCovariance()
{
    Eigen::Matrix<double, State::kNAugmentedState, State::kNAugmentedState> P;
    P.setZero();

    P.block<State::kNImuState, State::kNImuState>(0, 0) =
        Eigen::MatrixXd::Constant(State::kNImuState, State::kNImuState, 10e-3);

    const Eigen::Matrix<double, Config::kMaxPastElements * 6, 1>
        past_poses_cov_init =
            Eigen::Matrix<double, Config::kMaxPastElements * 6, 1>::Constant(
                0.0);

    Eigen::Matrix<double, State::kNBaseMultiWindowState, 1> imu_cov_init;

    imu_cov_init << 0.0011, 0.0011, 0.0011, 0.0011, 0.0011, 0.0011, 0.0011,
        0.01, 0.01, 0.01, 0.01, 0.01, 0.5, 0.5, 0.5,
        /*calib*/ 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001;

    Eigen::Matrix<double, State::kNAugmentedState, 1> diagonal;
    diagonal << imu_cov_init, past_poses_cov_init;
    P.diagonal() = diagonal;
    states_[state_index_].P_ = P;
}

State& RIO::getClosestState(const ros::Time& timestamp,
                            uint8_t& index_to_return, const double delay)
{
    uint8_t index = state_index_ - 1U;
    double time_distance = std::numeric_limits<double>::max();
    const double time_now = timestamp.toSec() - delay;
    while (std::abs(time_now - states_[index].time_) < time_distance)
    {
        // time_distance decreases continuously until best point
        // reached.
        time_distance = std::abs(time_now - states_[index].time_);
        --index;
    }
    // We subtracted one too many before.
    ++index;

    index_to_return = index;
    return states_[index];
}

void RIO::initializeStateFromConfig(const std::string& config_file)
{
    const YAML::Node config = YAML::LoadFile(config_file);
    State& initial_state = states_[state_index_];

    // Previous radar pose is not initialized.
    initial_state.p_ << config["px"].as<double>(), config["py"].as<double>(),
        config["pz"].as<double>();
    initial_state.v_ << config["vx"].as<double>(), config["vy"].as<double>(),
        config["vz"].as<double>();

    initial_state.q_ = Eigen::Quaternion<double>(
        config["qw"].as<double>(), config["qx"].as<double>(),
        config["qy"].as<double>(), config["qz"].as<double>());
    initial_state.q_.normalize();

    initial_state.b_w_ << config["b_wx"].as<double>(),
        config["b_wy"].as<double>(), config["b_wz"].as<double>();
    initial_state.b_a_ << config["b_ax"].as<double>(),
        config["b_ay"].as<double>(), config["b_az"].as<double>();

    initial_state.q_ri_ = Eigen::Quaternion<double>(
        config["q_riw"].as<double>(), config["q_rix"].as<double>(),
        config["q_riy"].as<double>(), config["q_riz"].as<double>());
    initial_state.q_ri_.normalize();

    initial_state.p_ri_ << config["p_rix"].as<double>(),
        config["p_riy"].as<double>(), config["p_riz"].as<double>();

    // Initialize time - the below timestamp is overwritten in the first
    // call to the IMU callback.
    initial_state.time_ = ros::Time::now().toSec();
    initial_state.w_m_ = state_predictor_.getGravityVector();
    initial_state.a_m_ = state_predictor_.getGravityVector();

    // Initialize parameters.
    parameters_.noise_acc_ = config["noise_acc"].as<double>();
    parameters_.noise_accbias_ = config["noise_accbias"].as<double>();
    parameters_.noise_gyr_ = config["noise_gyr"].as<double>();
    parameters_.noise_gyrbias_ = config["noise_gyrbias"].as<double>();
    parameters_.noise_qri_ = config["noise_qri"].as<double>();
    parameters_.noise_pri_ = config["noise_pri"].as<double>();
    parameters_.noise_meas1_ = config["noise_meas1"].as<double>();
    parameters_.noise_meas2_ = config["noise_meas2"].as<double>();
    parameters_.noise_meas3_ = config["noise_meas3"].as<double>();
    parameters_.noise_meas4_ = config["noise_meas4"].as<double>();

    parameters_.noise_pf_x_ = config["noise_pf_x"].as<double>();
    parameters_.noise_pf_y_ = config["noise_pf_y"].as<double>();
    parameters_.noise_pf_z_ = config["noise_pf_z"].as<double>();

    parameters_.noise_qwv_ = config["noise_qwv"].as<double>();
    parameters_.noise_aux_ = config["noise_aux"].as<double>();
    parameters_.noise_scale_ = config["noise_scale"].as<double>();
}

}  // namespace aaucns_rio
