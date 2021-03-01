//
// Created by Shahwaz Khan on 2/19/21.
//

#ifndef SRC_PACKAGE_EDGE_DETECTION_H
#define SRC_PACKAGE_EDGE_DETECTION_H

#include "point_cloud_utilities/pcl_utilities.h"


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudT;

class PackageTracking{

protected:
    /* data */
    std::string file_name_;
    std::string file_path_ = "/home/shahwaz/catkin_ws/src/my_pcl_tutorials/write_data";

public:
    PackageTracking();
    ros::NodeHandle nh_;

//  Point cloud tracking members
    ros::Publisher package_cloud_pub_;
    std::vector<double> cam_box_limits_;
    PointCloudT::Ptr box_filtered_cloud_ptr_;
    PointCloudT::Ptr mesh_cloud_ptr_;
    PointCloudT::Ptr final_cloud_created;

    double tolerance{};

    PointCloudT cloudIn_xz;





//  Camera members

    ros::Subscriber cam_pointcloud_sub_;
    std::string frame_id_;
    PointCloudT::Ptr cam_scene_cloud_ptr_;

//  Callback functions

    void pointCloudInfoCb(const sensor_msgs::PointCloud2& scene_cloud);

    void cloud_processing(PointCloudT& cloudIn);

//  Class Methods
    void trackEdge();
    void applyBoxFilter();

};
#endif //SRC_PACKAGE_EDGE_DETECTION_H