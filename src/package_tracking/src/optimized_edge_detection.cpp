//
// Created by shahwaz on 3/5/21.
//

#include "package_tracking/optimized_edge_detection.h"

EdgeTracking::EdgeTracking() {

    package_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2>("/edge_cloud", 1);
    camera_point_cloud_sub_ = node_.subscribe("two/depth/color/points", 1, &EdgeTracking::pointCloudCb, this);

    camera_scene_cloud_ptr_ = PointCloudT ::Ptr (new PointCloudT);
    box_filter_cloud_ptr_ = PointCloudT::Ptr (new PointCloudT);

    final_point_cloud_ = PointCloudT::Ptr (new PointCloudT);

}

void EdgeTracking::pointCloudCb(const sensor_msgs::PointCloud2& cloud_scene) {

    frame_id_ = cloud_scene.header.frame_id;
    pcl::fromROSMsg(cloud_scene, *camera_scene_cloud_ptr_);

    std::cout << "Number of points from camera: " << camera_scene_cloud_ptr_->points.size() << std::endl;

    optimizedTrackEdge();

}

void EdgeTracking::optimizedTrackEdge() {

    boxFilter();
    point_cloud_vector = cloudProcessing(*box_filter_cloud_ptr_);

    PCLUtilities::publishCloudToRviz(*final_point_cloud_, package_cloud_publisher_, frame_id_);
    std::cout << "Publishing to RViz...." << std::endl;

}

void EdgeTracking::boxFilter() {

    // Using a CropBox filter to extract the region of interest from the camera scene.
    pcl::CropBox<pcl::PointXYZRGB> box_filter;
    std::cout<<"Input Point Cloud has: "<<camera_scene_cloud_ptr_->points.size()<<std::endl;
    box_filter.setInputCloud(camera_scene_cloud_ptr_->makeShared());
    box_filter.setMin(Eigen::Vector4f(camera_box_limits_[0], camera_box_limits_[2], camera_box_limits_[4], 1.0));
    box_filter.setMax(Eigen::Vector4f(camera_box_limits_[1], camera_box_limits_[3], camera_box_limits_[5], 1.0));
    box_filter.filter(*box_filter_cloud_ptr_);
    std::cout<<"The box-filtered point cloud has: "<<box_filter_cloud_ptr_->points.size()<<std::endl;

}


std::vector<std::vector<double>> EdgeTracking::cloudProcessing(PointCloudT &cloudIn) {

    for (auto & cloud : cloudIn)
    {
        cloud.x = PCLUtilities::round(cloud.x);
        cloud.y = 0;
        cloud.z = cloud.z;

    }

    std::cout << "PointCloud after flooring down x has: " << cloudIn.size()
              << " data points" << std::endl;

    std::vector<std::vector<double>>pcl_vector_;

    for (auto & cloud : cloudIn)
    {
        auto *store_point = new std::vector<double>(3);
        (*store_point)[0] = cloud.x;
        (*store_point)[1] = cloud.y;
        (*store_point)[2] = cloud.z;

        pcl_vector_.push_back(*store_point);
        delete store_point;
    }

    std::cout << "PointCloud after storing in vectors has : " << pcl_vector_.size()
              << " data points" << std::endl;

    return pcl_vector_;
}









