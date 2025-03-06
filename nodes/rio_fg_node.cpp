#include <ros/ros.h>

#include "aaucns_rio/fg/rio_fg.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rio_fg_node");
    ros::NodeHandle nh("~");
    aaucns_rio::RIOFg rio_fg("/ti_mmwave/radar_scan_pcl",
                             "/mavros/imu/data_raw", "/pose",
                             "/aaucns_rio_state", "config_fg.yaml",
                             aaucns_rio::Config::kWriteFeaturesToFile, nh);

    while (ros::ok())
    {
        ros::spin();
    }
}
