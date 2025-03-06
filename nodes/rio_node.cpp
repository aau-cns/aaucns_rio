#include <ros/ros.h>

#include "aaucns_rio/config.h"
#include "aaucns_rio/rio.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rio_node");
    ros::NodeHandle nh("~");

    aaucns_rio::RIO rio("/ti_mmwave/radar_scan_pcl", "/mavros/imu/data_raw",
                        "/pose", "/aaucns_rio_state", "config.yaml",
                        aaucns_rio::Config::kWriteFeaturesToFile, nh);

    while (ros::ok())
    {
        ros::spin();
    }
}
