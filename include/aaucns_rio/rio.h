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

#ifndef _RIO_H_
#define _RIO_H_

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
#include "aaucns_rio/iteration.h"
#include "aaucns_rio/parameters.h"
#include "aaucns_rio/points_associator.h"
#include "aaucns_rio/state.h"
#include "aaucns_rio/state_predictor.h"
#include "aaucns_rio/state_updater.h"
#include "aaucns_rio/trail.h"
namespace aaucns_rio
{
class RIO
{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr std::size_t kNStatesInBuffer = 256;
    RIO(const std::string& pc2_topic, const std::string& imu_topic,
        const std::string& pose_topic, const std::string& state_topic,
        const std::string& config_filename, const bool dump_features,
        ros::NodeHandle& nh);

    bool initServiceCallback(std_srvs::SetBool::Request& /*request*/,
                             std_srvs::SetBool::Response& res);
    virtual ~RIO();

    State& getCurrentState()
    {
        return states_[static_cast<uint8_t>(state_index_ - 1)];
    }
    State& getStateAtIndex(const uint8_t index)
    {
        return states_[static_cast<uint8_t>(index)];
    }
    const State& getCurrentState() const
    {
        // Since `state_index_` points 2 states ahead the state with
        // current estimated values, this call returns the empty state to be
        // filled with information.
        return states_[static_cast<uint8_t>(state_index_ - 1)];
    }
    void outputPoseAtUpdate(const sensor_msgs::PointCloud2ConstPtr& msg);

    const State& getPreviousState() const
    {
        // Previous state is indexed by `state_index_ - 2` since
        // state_index_ points at the state which is yet not "filled"
        // with information and is 2 index ahead from the state
        // filled with "current" (newest) information.
        // Effectively, this call returns the current (newest) state
        // filled with estamated values.
        return states_[static_cast<uint8_t>(state_index_ - 2)];
    }
    State& getPreviousState()
    {
        return states_[static_cast<uint8_t>(state_index_ - 2)];
    }

    void shiftStateIndexForwards() { ++state_index_; }
    void increaseImuCallbackTimesCalledCounter()
    {
        ++imu_callback_times_called_;
    }
    // Make sure we received anough IMU messages that we can calculate
    // meaningful dt time difference.
    bool hasImuCallbackBeenCalledEnoughTimes()
    {
        if (imu_callback_times_called_ > 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    State& getClosestState(const ros::Time& timestamp, uint8_t& index_to_return,
                           const double delay);

    void PC2Callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void ImuCallback(const sensor_msgs::ImuConstPtr& msg);

   protected:
    virtual void writeState(const aaucns_rio::DoubleArrayStampedPtr& msg);
    virtual void writePose(
        const geometry_msgs::PoseWithCovarianceStampedPtr& msg);

   private:
    StatePredictor state_predictor_;
    StateUpdater state_updater_;
    Parameters parameters_;
    ros::Subscriber pc2_sub_;
    ros::Subscriber imu_sub_;
    ros::Publisher state_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher pose_at_update_pub_;
    Features features_;
    std::array<State, kNStatesInBuffer> states_;
    uint8_t state_index_ = 0;
    bool is_initialized_ = false;
    bool is_prediction_made_ = false;
    bool are_enough_states_collected_ = false;
    std::size_t imu_callback_times_called_ = 0;
    PointsAssociator points_associator_;
    Trail trail_;
    debug::Logger logger_;
    std::string config_filename_;
    std::string pc2_topic_;
    std::string imu_topic_;
    // Make sure to keep alive the external nh.
    ros::NodeHandle& nh_;
    ros::ServiceServer initialization_service_;

    void outputState(const sensor_msgs::PointCloud2ConstPtr& msg);
    void outputPose(const sensor_msgs::ImuConstPtr& msg);

    void setupSubscribers(const std::string& pc2_topic,
                          const std::string& imu_topic, ros::NodeHandle& nh);

    void predictToNewestState(const uint8_t closest_to_measurement_state_index);
    void initialize(const std::string& config_file);
    void initializeCovariance();
    void initializeStateFromConfig(const std::string& config_file);
};

}  // namespace aaucns_rio

#endif /* _RIO_H_ */
