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
        {"imu", "/mavros/imu/data_raw"},
        {"gt_pose", "/twins_cns4/vrpn_client/raw_pose"},
        {"pc2", "/ti_mmwave/radar_scan_pcl"}};
    // Make sure the bagfile is inside ~/.ros folder wherefrom the binary is
    // executed.
    const std::string input_bagfile = "awr_6.bag";
    aaucns_rio::RIOReplay rio_replay("config.yaml", topics_and_topic_names,
                                     input_bagfile, nh);
    rio_replay.run();
}
