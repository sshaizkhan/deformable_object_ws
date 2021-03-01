//
// Created by Shahwaz Khan on 2/23/21.
//
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>

using namespace std;
ros::Publisher pub;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudptr;
std::vector<double> cam_box_limits_;

//void applyBoxFilter()
//{
//    // Using a CropBox filter to extract the region of interest from the camera scene.
//
//    pcl::CropBox<pcl::PointXYZRGB> box_filter;
//    std::cout<<"Input Point Cloud has: "<<cam_scene_cloud_ptr_->points.size()<<std::endl;
//    box_filter.setInputCloud(cam_scene_cloud_ptr_->makeShared());
//    box_filter.setMin(Eigen::Vector4f(cam_box_limits_[0], cam_box_limits_[2], cam_box_limits_[4], 1.0));
//    box_filter.setMax(Eigen::Vector4f(cam_box_limits_[1], cam_box_limits_[3], cam_box_limits_[5], 1.0));
//    box_filter.filter(*box_filtered_cloud_ptr_);
//    std::cout<<"The box-filtered point cloud has: "<<box_filtered_cloud_ptr_->points.size()<<std::endl;
//
//    PCLUtilities::savePointCloudToPLY(*box_filtered_cloud_ptr_, file_path_, "/package_PLY_2.ply");
//}




void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
    cloudptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloudptr filtered_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud, *point_cloud);
    sensor_msgs::PointCloud2 cloud_filtered;

    pcl::CropBox<pcl::PointXYZRGB> cropBoxFilter (true);
    cropBoxFilter.setInputCloud(point_cloud);
    cropBoxFilter.setMin(Eigen::Vector4f(cam_box_limits_[0], cam_box_limits_[2], cam_box_limits_[4], 1.0));
    cropBoxFilter.setMax(Eigen::Vector4f(cam_box_limits_[1], cam_box_limits_[3], cam_box_limits_[5], 1.0));

    cropBoxFilter.filter(*filtered_point_cloud);


    pcl::toROSMsg(*filtered_point_cloud, cloud_filtered);
    cloud_filtered.header.frame_id = cloud->header.frame_id;

    pub.publish(cloud_filtered);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "surface_tracking");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("one/depth/color/points", 1, cloud_cb);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/crop_box", 1);

    ros::NodeHandle pnh("~");
    pnh.param("cam_bounding_box", cam_box_limits_, std::vector<double>());
    ros::spin();
}
