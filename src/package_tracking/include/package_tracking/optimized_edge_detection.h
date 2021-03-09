//
// Created by shahwaz on 3/5/21.
//

#ifndef PACKAGE_TRACKING_OPTIMIZED_EDGE_DETECTION_H
#define PACKAGE_TRACKING_OPTIMIZED_EDGE_DETECTION_H

#include "point_cloud_utilities/pcl_utilities.h"
#include "ros/package.h"
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudT;


class EdgePoint {
private:
    float x_coordinate;
    float min_z_coordinate;
    float corresponding_y;

public:
    float getXCoordinate() const {
        return x_coordinate;
    }

    float getMinZCoordinate() const {
        return min_z_coordinate;
    }

    float getCorrespondingY() const {
        return corresponding_y;
    }

    EdgePoint(float xCoordinate, float minZCoordinate, float correspondingY) : x_coordinate(xCoordinate),
                                                                               min_z_coordinate(minZCoordinate),
                                                                               corresponding_y(correspondingY) {}


};

class Coordinate{
private:
    float x;
    float y;
    float z;

public:
    float getX() const {
        return x;
    }

    float getY() const {
        return y;
    }

    float getZ() const {
        return z;
    }

    Coordinate(float x, float y, float z) : x(x), y(y), z(z) {}
};

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

//    Data members
    double tolerance{};

//  Camera members
    ros::Subscriber camera_point_cloud_sub_;
    std::string frame_id_;
    PointCloudT::Ptr camera_scene_cloud_ptr_;

//  Callback functions
    void pointCloudCb(const sensor_msgs::PointCloud2& cloud_scene);

//  Class Methods
    void optimizedTrackEdge();
    void boxFilter();
    static std::vector<Coordinate> cloudProcessing(PointCloudT& cloudIn);

    static std::vector<EdgePoint> finalEdgeTracking(const std::vector<Coordinate>& pclVector_);

    static pcl::PointCloud<pcl::PointXYZ>buildCloud(const std::vector<EdgePoint>& edgePoints);

};

#endif //PACKAGE_TRACKING_OPTIMIZED_EDGE_DETECTION_H
