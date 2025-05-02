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
#include "aaucns_rio/rio.h"
#include "aaucns_rio/rio_replay.h"

/*
This node opens a bagfile and loops through all messages and and manually calls
callback functions in order to avoid the network traffic causing processing
delays.
*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rio_replay_node");
    ros::NodeHandle nh("~");

    const std::map<std::string, std::string> topics_and_topic_names{
        // Output.
        {"state", "/aaucns_rio_state"},
        {"pose", "/pose"},
        // Input.
        {"imu", "/vectornav_node/uncomp_imu"},
        {"gt_pose", "/twins_cns4/vrpn_client/raw_pose"},
        {"pc2", "/radar/cloud"}};
    // Make sure the bagfile is inside ~/.ros folder wherefrom the binary is
    // executed.
    const std::string input_bagfile = "fog_filled_uni_corridor.bag";
    aaucns_rio::RIOReplay rio_replay("ntnu_config.yaml", topics_and_topic_names,
                                     input_bagfile, nh);
    rio_replay.run();
}
