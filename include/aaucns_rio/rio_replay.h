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

#ifndef _RIO_REPLAY_H_
#define _RIO_REPLAY_H_

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <map>
#include <string>
#include <vector>

#include "aaucns_rio/rio.h"

namespace aaucns_rio
{
class RIOReplay : public RIO
{
   public:
    // Open the bag and call run().
    RIOReplay(const std::string& config_filename,
              const std::map<std::string, std::string>& topic_names_and_topics,
              const std::string& input_bag, ros::NodeHandle& nh);
    ~RIOReplay();

    // Run the loop where callbacks are called explicitely at each iteration of
    // the bag.
    void run();

   protected:
    void writeState(const aaucns_rio::DoubleArrayStampedPtr& msg) override;
    void writePose(
        const geometry_msgs::PoseWithCovarianceStampedPtr& msg) override;

   private:
    ros::Publisher gt_pose_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher pc2_pub_;
    std::map<std::string, std::string> topic_names_and_topics_;
    rosbag::Bag input_bag_;
    rosbag::Bag output_bag_;
};

}  // namespace aaucns_rio

#endif /* _RIO_REPLAY_H_ */
