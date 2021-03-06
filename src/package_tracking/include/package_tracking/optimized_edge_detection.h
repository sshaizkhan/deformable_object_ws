//
// Created by shahwaz on 3/5/21.
//

#ifndef PACKAGE_TRACKING_OPTIMIZED_EDGE_DETECTION_H
#define PACKAGE_TRACKING_OPTIMIZED_EDGE_DETECTION_H

#include "point_cloud_utilities/pcl_utilities.h"
#include "ros/package.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudT;

class EdgeTracking{

protected:
    /* data */
    std::string file_path_ = ros::package::getPath("package_tracking") + "/data/";

public:
    EdgeTracking();
    ros::NodeHandle node_;

//  Point cloud tracking members
    ros::Publisher package_cloud_publisher_;
    std::vector<double> camera_box_limits_;
    PointCloudT::Ptr box_filter_cloud_ptr_;
    PointCloudT::Ptr final_point_cloud_;

//    Data members
    double tolerance{};
    std::vector<std::vector<double>>point_cloud_vector;

//  Camera members

    ros::Subscriber camera_point_cloud_sub_;
    std::string frame_id_;
    PointCloudT::Ptr camera_scene_cloud_ptr_;

//  Callback functions

    void pointCloudCb(const sensor_msgs::PointCloud2& cloud_scene);

    static std::vector<std::vector<double>> cloudProcessing(PointCloudT& cloudIn);

//  Class Methods
    void optimizedTrackEdge();

    void boxFilter();

};

#endif //PACKAGE_TRACKING_OPTIMIZED_EDGE_DETECTION_H
