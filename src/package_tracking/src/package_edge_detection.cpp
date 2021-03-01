//
// Created by Shahwaz Khan on 2/19/21.
//

#include "package_tracking/package_edge_detection.h"

PackageTracking::PackageTracking()
{

    package_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/package_cloud", 1);
    cam_pointcloud_sub_ = nh_.subscribe("two/depth/color/points", 1, &PackageTracking::pointCloudInfoCb, this);

    cam_scene_cloud_ptr_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    box_filtered_cloud_ptr_ = PointCloudT::Ptr (new PointCloudT);

    final_cloud_created = PointCloudT ::Ptr (new PointCloudT);


}

void PackageTracking::pointCloudInfoCb(const sensor_msgs::PointCloud2 &scene_cloud)
{
    frame_id_ = scene_cloud.header.frame_id;
    pcl::fromROSMsg(scene_cloud, *cam_scene_cloud_ptr_);

    std::cout << "Number of points from camera: " << cam_scene_cloud_ptr_->points.size() << std::endl;

    trackEdge();
}

void PackageTracking::trackEdge()
{


//    *cam_scene_cloud_ptr_ = PCLUtilities::downSampled(*cam_scene_cloud_ptr_, 0.05);
    applyBoxFilter();
    cloud_processing(*box_filtered_cloud_ptr_);
    PCLUtilities::publishMeshToRviz(*final_cloud_created, package_cloud_pub_, frame_id_);

    std::cout << "Publishing to RViz...." << std::endl;
}

void PackageTracking::applyBoxFilter()
{
    // Using a CropBox filter to extract the region of interest from the camera scene.

    pcl::CropBox<pcl::PointXYZRGB> box_filter;
    std::cout<<"Input Point Cloud has: "<<cam_scene_cloud_ptr_->points.size()<<std::endl;
    box_filter.setInputCloud(cam_scene_cloud_ptr_->makeShared());
    box_filter.setMin(Eigen::Vector4f(cam_box_limits_[0], cam_box_limits_[2], cam_box_limits_[4], 1.0));
    box_filter.setMax(Eigen::Vector4f(cam_box_limits_[1], cam_box_limits_[3], cam_box_limits_[5], 1.0));
    box_filter.filter(*box_filtered_cloud_ptr_);
    std::cout<<"The box-filtered point cloud has: "<<box_filtered_cloud_ptr_->points.size()<<std::endl;

    PCLUtilities::savePointCloudToPLY(*box_filtered_cloud_ptr_, file_path_, "/package_PLY_2.ply");
}

void PackageTracking::cloud_processing(PointCloudT& cloudIn)
{
    for (auto & i : cloudIn)
    {
        i.x = PCLUtilities::round(i.x);
    }

    std::cout << "PointCloud after flooring down x has: " << cloudIn.size()
              << " data points" << std::endl;

    std::vector<std::vector<double>>pcl_to_vector_;

    for (auto & i : cloudIn)
    {
        auto *store_point = new std::vector<double>(3);
        (*store_point)[0] = i.x;
        (*store_point)[1] = i.y;
        (*store_point)[2] = i.z;

        pcl_to_vector_.push_back(*store_point);
        delete store_point;
    }

    std::cout << "PointCloud after storing in vectors has : " << pcl_to_vector_.size()
              << " data points" << std::endl;


    std::vector<double>all_z_vector;

    for(auto & i : pcl_to_vector_)
    {
        double z;
        z = i[2];
        all_z_vector.push_back(z);
    }

    sort(all_z_vector.begin(), all_z_vector.end());
    std::cout << "All Z Vector after storing in only z has : " << all_z_vector.size()
              << " data points" << std::endl;


    final_cloud_created->width = pcl_to_vector_.size();
    final_cloud_created->height = 1;
    final_cloud_created->points.resize(final_cloud_created->width * final_cloud_created->height);

    tolerance = 0.03;

    for (std::size_t i = 0; i < final_cloud_created->points.size(); i++)
    {
        if (pcl_to_vector_[i][2] >= all_z_vector[0]&& pcl_to_vector_[i][2] <= all_z_vector[0] + tolerance)
        {
            final_cloud_created->points[i].x = pcl_to_vector_[i][0];
            final_cloud_created->points[i].y = pcl_to_vector_[i][1];
            final_cloud_created->points[i].z = pcl_to_vector_[i][2];
        }
        else
            continue;
    }
    std::cout << "PointCloud after creating from vectors has : " << final_cloud_created->points.size()
              << " data points" << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "edge_tracking");
    PackageTracking packObj;
    ros::NodeHandle pnh("~");
    pnh.param("cam_bounding_box", packObj.cam_box_limits_, std::vector<double>());
    ros::Rate loop_rate(30);


    while (ros::ok())
    {
        ros::spin();
    }
    return 0;
}