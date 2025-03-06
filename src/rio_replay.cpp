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

#include "aaucns_rio/rio_replay.h"

#include <aaucns_rio/DoubleArrayStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/SetBool.h>

#include <algorithm>
#include <iterator>
namespace aaucns_rio
{
RIOReplay::RIOReplay(
    const std::string& config_filename,
    const std::map<std::string, std::string>& topic_names_and_topics,
    const std::string& input_bag, ros::NodeHandle& nh)
    : RIO(topic_names_and_topics.at("pc2"), topic_names_and_topics.at("imu"),
          topic_names_and_topics.at("pose"), topic_names_and_topics.at("state"),
          config_filename, Config::kWriteFeaturesToFile, nh),
      topic_names_and_topics_(topic_names_and_topics)
{
    // Below just a dummy call - real service only needed when used with
    // the flight stack.
    std_srvs::SetBool srv;
    (void)initServiceCallback(srv.request, srv.response);
    input_bag_.open(input_bag, rosbag::bagmode::Read);
    output_bag_.open(std::string("replayed_") + input_bag,
                     rosbag::bagmode::Write);
}

void RIOReplay::writeState(const aaucns_rio::DoubleArrayStampedPtr& msg)
{
    output_bag_.write(topic_names_and_topics_.at("state"), msg->header.stamp,
                      msg);
};

void RIOReplay::writePose(
    const geometry_msgs::PoseWithCovarianceStampedPtr& msg)
{
    output_bag_.write(topic_names_and_topics_.at("pose"), msg->header.stamp,
                      msg);
};

RIOReplay::~RIOReplay()
{
    input_bag_.close();
    output_bag_.close();
}

void RIOReplay::run()
{
    std::vector<std::string> topics;
    std::transform(topic_names_and_topics_.begin(),
                   topic_names_and_topics_.end(), std::back_inserter(topics),
                   [](auto& kv) { return kv.second; });

    rosbag::TopicQuery query(topics);
    rosbag::View view(input_bag_, query);

    for (const rosbag::MessageInstance m : view)
    {
        if (!ros::ok())
        {
            std::cerr << "Aborting." << std::endl;
            break;
        }

        sensor_msgs::PointCloud2ConstPtr pc2 =
            m.instantiate<sensor_msgs::PointCloud2>();
        sensor_msgs::ImuConstPtr imu = m.instantiate<sensor_msgs::Imu>();
        geometry_msgs::PoseStamped::ConstPtr gt_pose =
            m.instantiate<geometry_msgs::PoseStamped>();

        // Call callbacks for imu and radar and do write-back on
        // all other messages so that the timestamps are in the same range for
        // all.
        if (imu != nullptr &&
            (m.getTopic() == topic_names_and_topics_.at("imu")))
        {
            ImuCallback(imu);
            // Write back imu with new timestamp.
            output_bag_.write(topic_names_and_topics_.at("imu"),
                              imu->header.stamp, m);
        }
        if (pc2 != nullptr &&
            (m.getTopic() == topic_names_and_topics_.at("pc2")))
        {
            PC2Callback(pc2);
            // Write back pc2 with new timestamp.
            output_bag_.write(topic_names_and_topics_.at("pc2"),
                              pc2->header.stamp, m);
        }
        if (gt_pose != nullptr &&
            (m.getTopic() == topic_names_and_topics_.at("gt_pose")))
        {
            // Write back gt_pose with new timestamp.
            output_bag_.write(topic_names_and_topics_.at("gt_pose"),
                              gt_pose->header.stamp, m);
        }

        ros::spinOnce();
    }
    std::cout << "Loop ended." << std::endl;
}

}  // namespace aaucns_rio