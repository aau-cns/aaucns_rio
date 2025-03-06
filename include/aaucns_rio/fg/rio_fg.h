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

#ifndef _RIO_FG_H_
#define _RIO_FG_H_

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PreintegrationCombinedParams.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/SetBool.h>

#include <Eigen/Dense>
#include <array>
#include <cstddef>
#include <fstream>
#include <string>

#include "aaucns_rio/circular_buffer.h"
#include "aaucns_rio/debug.h"
#include "aaucns_rio/features.h"
#include "aaucns_rio/fg/factors_data.h"
#include "aaucns_rio/fg/imu_measurement.h"
#include "aaucns_rio/fg/marginalization.h"
#include "aaucns_rio/parameters.h"
#include "aaucns_rio/points_associator.h"
#include "aaucns_rio/trail.h"
namespace aaucns_rio
{
class RIOFg
{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RIOFg(const std::string& pc2_topic, const std::string& imu_topic,
          const std::string& pose_topic, const std::string& state_topic,
          const std::string& config_filename, const bool dump_features,
          ros::NodeHandle& nh);

    void increasePC2CallbackTimesCalledCounter()
    {
        ++pc2_callback_times_called_;
    }
    // Make sure we received anough IMU messages that we can calculate
    // meaningful dt time difference.
    bool hasPC2CallbackBeenCalledEnoughTimes()
    {
        if (pc2_callback_times_called_ > 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool initServiceCallback(std_srvs::SetBool::Request& /*request*/,
                             std_srvs::SetBool::Response& res);
    virtual ~RIOFg();

    std::shared_ptr<gtsam::PreintegratedCombinedMeasurements>
    preintegrateIMUMeasurements(const double ti_s, const double tf_s);

    IMUMeasurement& getClosestIMUMeasurement(const double timestamp_s,
                                             int& index_to_return,
                                             const double delay);

    void PC2Callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void IMUCallback(const sensor_msgs::ImuConstPtr& msg);

    gtsam::Values solveGraph();
    void initializeParameters();

    void initializePFsValues();

    void releasePFsIds();
    void initializeSolution();

    void marginalize();

   protected:
    virtual void writeState(const aaucns_rio::DoubleArrayStampedPtr& msg);
    virtual void writePose(
        const geometry_msgs::PoseWithCovarianceStampedPtr& msg);

   private:
    Parameters parameters_;
    ros::Subscriber pc2_sub_;
    ros::Subscriber imu_sub_;
    ros::Publisher state_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher pose_at_update_pub_;
    gtsam::Pose3 initial_pose_;
    gtsam::Vector3 initial_velocity_;
    gtsam::NavState initial_state_;
    gtsam::NavState previous_state_;
    gtsam::imuBias::ConstantBias initial_imu_bias_;
    gtsam::noiseModel::Gaussian::shared_ptr prior_pose_noise_;
    gtsam::noiseModel::Gaussian::shared_ptr prior_velocity_noise_;
    gtsam::noiseModel::Gaussian::shared_ptr prior_bias_noise_;
    gtsam::noiseModel::Gaussian::shared_ptr prior_pf_noise_;
    gtsam::Pose3 radar_imu_transform_;
    gtsam::Vector noise_model_bias_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values values_;
    gtsam::Values previous_solution_;
    std::shared_ptr<gtsam::PreintegrationCombinedParams> preintegration_params_;
    Marginalization marginalization_;

    // The below state holds persistent features.
    State most_recent_updated_state_;
    CircularBuffer<IMUMeasurement, ConfigFg::kNMaxIMUMeasurements>
        imu_measurements_;

    CircularBuffer<IMUFactorData, ConfigFg::kNPosesInWindow>
        imu_factor_data_horizon_;
    CircularBuffer<RadialVelocityFactorData, ConfigFg::kNPosesInWindow>
        velocity_factor_data_horizon_;
    CircularBuffer<RadarDistanceFactorData, ConfigFg::kNPosesInWindow>
        distance_factor_data_horizon_;
    CircularBuffer<RadarPFDistanceFactorData, ConfigFg::kNPosesInWindow>
        pf_factor_data_horizon_;

    gtsam::Marginals marginals_;
    gtsam::ISAM2 optimizer_;
    Features features_;
    bool is_initialized_ = false;
    PointsAssociator points_associator_;
    Trail trail_;
    debug::Logger logger_;
    std::string config_filename_;
    std::string pc2_topic_;
    std::string imu_topic_;
    // Make sure to keep alive the external nh.
    ros::NodeHandle& nh_;
    ros::ServiceServer initialization_service_;
    std::size_t pc2_callback_times_called_ = 0;
    double previous_pc2_timestamp_s_;
    int poses_in_window_ = 0;

    void outputPose(const double ti_f, const uint32_t seq);

    void setupSubscribers(const std::string& pc2_topic,
                          const std::string& imu_topic, ros::NodeHandle& nh);

    void initialize(const std::string& config_file);
    void initializeStateFromConfig(const std::string& config_file);
};

}  // namespace aaucns_rio

#endif /* _RIO_FG_H_ */
