//
// Created by Shahwaz Khan on 2/19/21.
// shahwazk@usc.edu
//

#include "package_tracking/package_edge_detection.h"

PackageTracking::PackageTracking()
{
    package_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/package_cloud", 1);
    cam_pointcloud_sub_ = nh_.subscribe("two/depth/color/points", 1, &PackageTracking::pointCloudInfoCb, this);

    cam_scene_cloud_ptr_ = PointCloudT::Ptr (new PointCloudT);
    box_filtered_cloud_ptr_ = PointCloudT::Ptr (new PointCloudT);



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

    applyBoxFilter();
    PointCloudT::Ptr euclidean_point_cloud_  (new PointCloudT);
    PointCloudT ::Ptr processed_cloud_ (new PointCloudT);
//    *euclidean_point_cloud_ = PCLUtilities::euclideanClustering( box_filtered_cloud_ptr_, ec_tolerance, minClusterSize, maxClusterSize);

    *processed_cloud_ = cloud_processing(*box_filtered_cloud_ptr_);

    PCLUtilities::publishPCLToRviz(*processed_cloud_, package_cloud_pub_, frame_id_);
//    PCLUtilities::savePointCloudToPLY(*final_cloud_created, file_path_, file_name + ".ply");
    std::cout << "Publishing to RViz...." << std::endl;

}

void PackageTracking::applyBoxFilter()
{
    // Using a CropBox filter to extract the region of interest from the camera scene.

    pcl::CropBox<pcl::PointXYZ> box_filter;
    std::cout<<"Input Point Cloud has: "<<cam_scene_cloud_ptr_->points.size()<<std::endl;
    box_filter.setInputCloud(cam_scene_cloud_ptr_->makeShared());
    box_filter.setMin(Eigen::Vector4f(cam_box_limits_[0], cam_box_limits_[2], cam_box_limits_[4], 1.0));
    box_filter.setMax(Eigen::Vector4f(cam_box_limits_[1], cam_box_limits_[3], cam_box_limits_[5], 1.0));
    box_filter.filter(*box_filtered_cloud_ptr_);
    std::cout<<"The box-filtered point cloud has: "<<box_filtered_cloud_ptr_->points.size()<<std::endl;

}

PointCloudT PackageTracking::cloud_processing(PointCloudT& cloudIn) const
{
    std::vector<std::vector<double>>pcl_to_vector_;

    for (auto & cloud : cloudIn)
    {
        auto *store_point = new std::vector<double>(3);
        (*store_point)[0] = PCLUtilities::round(cloud.x, 0.005);
        (*store_point)[1] = cloud.y;
        (*store_point)[2] = cloud.z;

           pcl_to_vector_.push_back(*store_point);
        delete store_point;
    }

    std::cout << "PointCloud after storing in vectors has : " << pcl_to_vector_.size()
              << " data points" << std::endl;


    std::vector<double>all_z_vector;

    for(auto & pcl_vector : pcl_to_vector_)
    {
        all_z_vector.push_back(pcl_vector[2]);
    }

    sort(all_z_vector.begin(), all_z_vector.end());
    std::cout << "All Z Vector after storing in only z has : " << all_z_vector.size()
              << " data points" << std::endl;

    PointCloudT::Ptr cloud_created (new PointCloudT);
    cloud_created->width = pcl_to_vector_.size();
    cloud_created->height = 1;
    cloud_created->points.resize(cloud_created->width * cloud_created->height);

    for (std::size_t i = 0; i < cloud_created->points.size(); i++)
    {
        if (pcl_to_vector_[i][2] >= all_z_vector[0]&& pcl_to_vector_[i][2] <= all_z_vector[0] + tolerance)
        {
            cloud_created->points[i].x = pcl_to_vector_[i][0];
            cloud_created->points[i].y = pcl_to_vector_[i][1];
            cloud_created->points[i].z = pcl_to_vector_[i][2];
        }
        else
            continue;
    }
    std::cout << "PointCloud after creating from vectors has : " << cloud_created->points.size()
              << " data points" << std::endl;

    return *cloud_created;

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "edge_tracking");
    PackageTracking packObj;
    ros::NodeHandle pnh("~");
    pnh.param("cam_bounding_box", packObj.cam_box_limits_, std::vector<double>());
    pnh.param("file_name", packObj.file_name, std::string());
    pnh.param("z_tolerance", packObj.tolerance, double());
    pnh.param("ec_tolerance", packObj.ec_tolerance, double());
    pnh.param("minClusterSize", packObj.minClusterSize, int());
    pnh.param("maxClusterSize", packObj.maxClusterSize, int());
    ros::Rate loop_rate(5);


    while (ros::ok())
    {
        ros::spin();
    }
    return 0;
}