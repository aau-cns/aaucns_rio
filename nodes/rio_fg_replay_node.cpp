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

#include <ros/ros.h>

#include <map>
#include <string>

#include "aaucns_rio/config.h"
#include "aaucns_rio/fg/rio_fg.h"
#include "aaucns_rio/fg/rio_fg_replay.h"

/*
This node opens a bagfile and loops through all messages and and manually calls
callback functions in order to avoid the network traffic causing processing
delays.
*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rio_fg_replay_node");
    ros::NodeHandle nh("~");

    const std::map<std::string, std::string> topics_and_topic_names{
        // Output.
        {"state", "/aaucns_rio_state"},
        {"pose", "/pose"},
        // Input.
        {"imu", argc > 3 ? argv[3] : "/mavros/imu/data_raw"},
        {"gt_pose", "/twins_cns4/vrpn_client/raw_pose"},
        {"pc2", "/ti_mmwave/radar_scan_pcl"}};
    // Make sure the bagfile is inside ~/.ros folder wherefrom the binary is
    // executed.
    // Optional CLI overrides: <input_bag> [config_file] [imu_topic].
    const std::string input_bagfile = argc > 1 ? argv[1] : "awr_7.bag";
    const std::string config_file = argc > 2 ? argv[2] : "config.yaml";
    aaucns_rio::RIOFgReplay rio_fg_replay(config_file, topics_and_topic_names,
                                          input_bagfile, nh);
    rio_fg_replay.run();
}
