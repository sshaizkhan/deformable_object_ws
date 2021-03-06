//
// Created by shahwazk@usc.edu on 1/29/21.
//

#ifndef SRC_PCL_UTILITIES_H
#define SRC_PCL_UTILITIES_H
/****************************************/
//          ROS HEADERS
/****************************************/
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

/****************************************/
//          PCL HEADERS
/****************************************/
#include <pcl/common/common_headers.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include "pcl/point_types.h"
#include "pcl/filters/crop_box.h"
#include "pcl/impl/point_types.hpp"
#include "pcl/filters/voxel_grid.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/features/normal_3d.h"
#include "pcl/surface/gp3.h"
#include "pcl/io/vtk_io.h"
#include "pcl/io/vtk_lib_io.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/io/ply_io.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/registration/icp.h"
#include "pcl/visualization/cloud_viewer.h"
#include "pcl/compression/octree_pointcloud_compression.h"
#include "pcl/io/openni_grabber.h"
#include "pcl/octree/octree_search.h"
#include "pcl/filters/passthrough.h"

/****************************************/
//          STANDARD HEADERS
/****************************************/
#include "iostream"
#include "cmath"
#include "vector"
#include "utility"
#include "chrono"
#include "unordered_map"
#include "queue"
#include "fstream"
#include "thread"
#include "ctime"
#include "random"
#include "typeinfo"
#include "iomanip"

/****************************************/
//          OPENCV HEADERS
/****************************************/
#include "opencv/cv.h"
#include "opencv/highgui.h"

/****************************************/
//          EIGEN HEADERS
/****************************************/
#include <Eigen/Dense>

/****************************************/
//          NAME SPACE
/****************************************/

namespace PCLUtilities
{
    float round(float var)
    {
        // 37.66666 * 100 =3766.66
        // 3766.66 + .5 =3767.16    for rounding off value
        // then type cast to int so value is 3767
        // then divided by 100 so the value converted into 37.67
        float value = (int)(var * 1000 + .005);
        return (float)value / 1000;
    }

    template<typename PointT>
    inline void publishPCLToRviz(pcl::PointCloud<PointT>cloudIn, ros::Publisher& pub, std::string& frame_id)
    {
        sensor_msgs::PointCloud2 mesh_cloud_msg;
        pcl::toROSMsg(cloudIn, mesh_cloud_msg);
        mesh_cloud_msg.header.frame_id = frame_id;
        std::cout<<"Mesh to be published has: " << cloudIn.points.size() << std::endl;
        pub.template publish(mesh_cloud_msg);

    }

    template<typename PointT>
    void savePointCloudToPLY(pcl::PointCloud<PointT>cloudIn,const std::string& file_path, const std::string& file_name)
    {
        pcl::io::savePLYFileASCII(file_path + file_name, cloudIn);
        std::cerr << "Saved " << file_name << " at location: " << file_path <<std::endl;
    }

    template<typename PointT>
    void savePointCloudToPCD(pcl::PointCloud<PointT>cloudIn, const std::string& file_path, const std::string& file_name)
    {
        pcl::io::savePCDFileASCII(file_path + file_name, cloudIn);
        std::cerr << "Saved " << file_name << "at location: " << file_path <<std::endl;

    }

    template<typename PointT>
    pcl::PointCloud<PointT>readPCDFileToPointCloud(const std::string& file_path)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_out_ (new pcl::PointCloud<PointT>);

        if(pcl::io::loadPCDFile(file_path, *cloud_out_) == -1)
        {
            PCL_ERROR("Couldn't read the PCD File \n");

        }
        std::cout << "Loaded: " << cloud_out_->width * cloud_out_->height
                  << " data points from the loaded PLY file" <<std::endl;

        return *cloud_out_;
    }

    template<typename PointT>
    pcl::PointCloud<PointT>readPLYFileToPointCloud(const std::string& file_path, const std::string& file_name)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_out_ (new pcl::PointCloud<PointT>);

        if(pcl::io::loadPLYFile(file_path + file_name, *cloud_out_) == -1)
        {
            PCL_ERROR("Couldn't read the PLY File \n");

        }
        std::cout << "Loaded: " << cloud_out_->width * cloud_out_->height
                                << " data points from the loaded PLY file" <<std::endl;

        return *cloud_out_;
    }


    template<typename PointT>
    pcl::PointCloud<PointT>outliersRemoval(pcl::PointCloud<PointT> cloud, int meanK, double standDevThresh)
    {
        pcl::PointCloud<PointT> cloud_filtered_;
        pcl::StatisticalOutlierRemoval<PointT> stat_filtered_;
        stat_filtered_.setInputCloud(cloud.makeShared());
        stat_filtered_.setMeanK(meanK);
        stat_filtered_.setStddevMulThresh(standDevThresh);
        stat_filtered_.filter(cloud_filtered_);

        return cloud_filtered_;

    }


    pcl::PointCloud<pcl::PointXYZ>makePointCloud(unsigned int cloudWidth)
    {
        pcl::PointCloud<pcl::PointXYZ>cloud_rendered_;
        cloud_rendered_.width = cloudWidth;
        cloud_rendered_.height = 1;
        cloud_rendered_.points.resize(cloudWidth*cloud_rendered_.height);

        float rand_number = 1024;

        for(auto & point : cloud_rendered_.points)
        {
            point.x = rand_number * rand() / (RAND_MAX + 1.0f);
            point.y = rand_number * rand() / (RAND_MAX + 1.0f);
            point.z = rand_number * rand() / (RAND_MAX + 1.0f);
        }

        return cloud_rendered_;

    }

    template<typename PointT>
    static pcl::visualization::PCLVisualizer visualizePointCloud(const pcl::PointCloud<PointT>& cloud)
    {
        pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addPointCloud<PointT>(cloud.makeShared(), "sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 1, "sample cloud");
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();

        return *viewer;

    }

    template<typename PointT>
    pcl::PointCloud<PointT>passThroughCloudFilter(pcl::PointCloud<PointT> cloudIn,
                                                  const std::string& filterFieldName,
                                                  const float lowerLimit, const float upperLimit)
    {
//        Pass-through filter
        typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
        typename pcl::PassThrough<PointT> pass;
        pass.setInputCloud(cloudIn.makeShared());
        pass.setFilterFieldName(filterFieldName);
        pass.setFilterLimits(lowerLimit, upperLimit);
        pass.filter(*cloud_filtered);

        return *cloud_filtered;
    }

    /// <summary>
    /// The function will down sample the point cloud of type PointT in all three dimensions.
    /// PointT can be of type like this: PointXYZ, PointXYZI, PointXYZRGB
    /// </summary>

    template<typename PointT>
    pcl::PointCloud<PointT>voxelGridFilter(pcl::PointCloud<PointT>cloudIn, const float leafX, const float leafY,const float leafZ)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
        typename pcl::VoxelGrid<PointT>voxel_sampler;
        voxel_sampler.setInputCloud(cloudIn.makeShared());
        voxel_sampler.setLeafSize(leafX, leafY, leafZ);
        voxel_sampler.filter(*cloud_filtered);

        return *cloud_filtered;
    }

    template<typename PointT>
    pcl::PointCloud<PointT>statisticalOutlierRemoval(pcl::PointCloud<PointT>cloudIn, float MeanK, float stdDevMulThreshold)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
        typename pcl::StatisticalOutlierRemoval<PointT>::Ptr stat_filter (new pcl::StatisticalOutlierRemoval<PointT>);
        stat_filter->setInputCloud(cloudIn.makeShared());
        stat_filter->setMeanK(MeanK);
        stat_filter->setStddevMulThresh(stdDevMulThreshold);
        stat_filter->filter(*cloud_filtered);

        return *cloud_filtered;
    }

    template<typename PointT>
    pcl::PointCloud<PointT>cloudMatching(pcl::PointCloud<PointT>cloudIn, pcl::PointCloud<PointT>cloudOut)
    {
        pcl::PointCloud<PointT>cloudAligned;

        for (size_t i = 0; i < cloudIn.points.size(); i++)
        {
            cloudOut.points[i].x = cloudIn.points[i].x + 0.7f;
        }

        pcl::IterativeClosestPoint<PointT, PointT> icp;
        icp.setInputSource(cloudIn.makeShared());
        icp.setInputTarget(cloudOut.makeShared());

        icp.setMaxCorrespondenceDistance(5);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-12);
        icp.setEuclideanFitnessEpsilon(0.1);

        icp.align(cloudAligned);

        std::cout << "has converged:" << icp.hasConverged() << " score: " <<icp.getFitnessScore() << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;

        return cloudAligned;

    }
//Error in catkin_make
    template<typename PointT>
    pcl::PointCloud<PointT>octreeCompression(pcl::PointCloud<PointT>& cloud)
    {
        typename pcl::PointCloud<PointT>::Ptr decompressedCloud;
        pcl::io::OctreePointCloudCompression<PointT>octreeCompress (pcl::io::MED_RES_OFFLINE_COMPRESSION_WITH_COLOR, true);
        std::stringstream compressionData;
        octreeCompress.encodePointCloud(cloud.makeShared(), compressionData);
        octreeCompress.decodePointCloud(compressionData, decompressedCloud);

        return *decompressedCloud;
    }


    template<typename PointT>
    pcl::PointCloud<PointT>applyBoxFilter(pcl::PointCloud<PointT>&cloudIn, std::vector<double>boxLimits)
    {
        typename pcl::CropBox<PointT>::Ptr box_filter_ (new pcl::CropBox<PointT>);
        typename pcl::PointCloud<PointT>::Ptr bodyFiltered_ (new pcl::PointCloud<PointT>);
        std::cout << "Input Point Cloud has: " <<cloudIn.points.size() << std::endl;
        box_filter_->setInputCloud(cloudIn.makeShared());
        box_filter_->setMin(Eigen::Vector4f(boxLimits[0], boxLimits[2], boxLimits[4], 1.0));
        box_filter_->setMax(Eigen::Vector4f(boxLimits[1], boxLimits[3], boxLimits[5], 1.0));
        box_filter_->filter(*bodyFiltered_);

        return *bodyFiltered_;

    }

    template<typename PointT, typename Value>
    pcl::PointCloud<PointT>octreeSearchPartition(pcl::PointCloud<PointT>& pointCloud, const std::string& searchType,
                                                 const double searchX, const double searchY, const double searchZ ,Value num)
    {
        float resolution = 128.0f;

        pcl::PointCloud<PointT> cloudPartitioned;
        pcl::octree::OctreePointCloudSearch<PointT> octree (resolution);
        octree.setInputCloud(pointCloud.makeShared());
        octree.addPointsFromInputCloud();

        PointT searchPoint;
        searchPoint.x = searchX;
        searchPoint.y = searchY;
        searchPoint.z = searchZ;
//         Neighbors within voxel search

        if (searchType == "NVS")
        {
            std::vector<int> pointIdxVec;
            if(octree.voxelSearch(searchPoint, pointIdxVec))
            {
                std::cout << "Neighbors within voxel search at (" << searchPoint.x
                          << " " << searchPoint.y
                          << " " << searchPoint.z << ")"
                          << std::endl;

                for (std::size_t i = 0; i < pointIdxVec.size (); ++i)
                {
                    std::cout << "    " << (pointCloud)[pointIdxVec[i]].x
                              << " " << (pointCloud)[pointIdxVec[i]].y
                              << " " << (pointCloud)[pointIdxVec[i]].z << std::endl;

                    cloudPartitioned.points.push_back(pointCloud.points[pointIdxVec[i]]);
                }
            }
        }
        else if (searchType =="KNN")
        {
            int K = num;
            std::cout << "Type of Resolution: " << typeid(num).name() << std::endl;
            std::vector<int> pointIdxNKNSearch;
            std::vector<float> pointNKNSquaredDistance;

            std::cout << "K nearest neighbor search at (" << searchPoint.x
                      << " " << searchPoint.y
                      << " " << searchPoint.z
                      << ") with K=" << K << std::endl;

            if (octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
            {
                for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
                {
                    cloudPartitioned.points.push_back(pointCloud.points[pointIdxNKNSearch[i]]);
                    std::cout << "    "  <<   (pointCloud)[ pointIdxNKNSearch[i] ].x
                              << " " << (pointCloud)[ pointIdxNKNSearch[i] ].y
                              << " " << (pointCloud)[ pointIdxNKNSearch[i] ].z
                              << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;


                }
            }
        }

        else if (searchType == "NRS")
        {
            float radius = num;
            std::cout << "Type of Resolution: " << typeid(num).name() << std::endl;

            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;

            std::cout << "Neighbors within radius search at (" << searchPoint.x
                      << " " << searchPoint.y
                      << " " << searchPoint.z
                      << ") with radius=" << radius << std::endl;

            if (octree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
            {
                std::cout<<"NRS: " << pointIdxRadiusSearch.size() << std::endl;
                for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i) {
                    std::cout << "    " << (pointCloud)[pointIdxRadiusSearch[i]].x
                              << " " << (pointCloud)[pointIdxRadiusSearch[i]].y
                              << " " << (pointCloud)[pointIdxRadiusSearch[i]].z
                              << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;

                    cloudPartitioned.points.push_back(pointCloud.points[pointIdxRadiusSearch[i]]);
                }
            }
        }

        return cloudPartitioned;

    }
};

#endif //SRC_PCL_UTILITIES_H
