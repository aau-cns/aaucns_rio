#include "aaucns_rio/pcl_conversions_rio.h"

#include <algorithm>
#include <set>
#include <string>

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(aaucns_rio::RadarPointCloudType,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, doppler,   doppler)
    )
// clang-format on

namespace aaucns_rio
{
bool rosPCLMsgToRadarPCL(const sensor_msgs::PointCloud2& ros_msg_pcl,
                         pcl::PointCloud<RadarPointCloudType>& radar_pcl)
{
    std::set<std::string> fields;
    std::for_each(ros_msg_pcl.fields.cbegin(), ros_msg_pcl.fields.cend(),
                  [&fields](const auto& field) { fields.emplace(field.name); });

    bool success = false;
    const bool has_xyzi = fields.find("x") != fields.end() &&
                          fields.find("y") != fields.end() &&
                          fields.find("z") != fields.end() &&
                          fields.find("intensity") != fields.end();
    if (has_xyzi && fields.find("doppler") != fields.end())
    {
        ROS_INFO_ONCE(
            "[Point cloud conversion]: Detected coloradar pcl format.");
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(ros_msg_pcl, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, radar_pcl);
        success = true;
    }
    else if (has_xyzi)
    {
        ROS_INFO_ONCE(
            "[Point cloud conversion]: Detected aaucns_rio pcl format.");
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(ros_msg_pcl, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZI> pcl_xyzi;
        pcl::fromPCLPointCloud2(pcl_pc2, pcl_xyzi);
        xyziToRadarPCL(pcl_xyzi, radar_pcl);
        success = true;
    }
    else
    {
        ROS_ERROR_STREAM(
            "[Point cloud conversion]: Unsupported point cloud format.");
    }
    return success;
}

void xyziToRadarPCL(const pcl::PointCloud<pcl::PointXYZI>& pcl_xyzi,
                    pcl::PointCloud<RadarPointCloudType>& radar_pcl)
{
    radar_pcl.header = pcl_xyzi.header;
    radar_pcl.width = pcl_xyzi.width;
    radar_pcl.height = pcl_xyzi.height;
    radar_pcl.is_dense = pcl_xyzi.is_dense == 1;
    radar_pcl.points.resize(pcl_xyzi.width);
    for (int i = 0; i < pcl_xyzi.width; ++i)
    {
        radar_pcl.points[i].x = pcl_xyzi.points[i].data[0];
        radar_pcl.points[i].y = pcl_xyzi.points[i].data[1];
        radar_pcl.points[i].z = pcl_xyzi.points[i].data[2];
        radar_pcl.points[i].doppler = pcl_xyzi.points[i].data[3];
        radar_pcl.points[i].intensity = pcl_xyzi.points[i].intensity;
    }
}

}  // namespace aaucns_rio