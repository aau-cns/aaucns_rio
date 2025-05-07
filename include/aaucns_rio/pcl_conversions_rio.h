#ifndef _PCL_CONVERSIONS_RIO_H_
#define _PCL_CONVERSIONS_RIO_H_

#include <pcl/PCLPointCloud2.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

namespace aaucns_rio
{
struct RadarPointCloudType
{
    PCL_ADD_POINT4D;  // position in [m]
    float intensity;  // CFAR cell to side noise ratio in [dB]
    float doppler;    // Doppler velocity in [m/s]
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
struct RadarPointCloudNTNUType
{
    PCL_ADD_POINT4D;  // position in [m]
    float intensity;  // CFAR cell to side noise ratio in [dB]
    float velocity;   // Doppler velocity in [m/s]
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

bool rosPCLMsgToRadarPCL(const sensor_msgs::PointCloud2& ros_msg_pcl,
                         pcl::PointCloud<RadarPointCloudType>& radar_pcl);

void xyziToRadarPCL(const pcl::PointCloud<pcl::PointXYZI>& pcl_xyzi,
                    pcl::PointCloud<RadarPointCloudType>& radar_pcl);

void NTNUToRadarPCL(
    const pcl::PointCloud<RadarPointCloudNTNUType>& ntnu_point_cloud,
    pcl::PointCloud<RadarPointCloudType>& radar_pcl);

}  // namespace aaucns_rio

#endif /* _PCL_CONVERSIONS_RIO_H_ */
