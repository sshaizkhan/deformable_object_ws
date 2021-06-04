//
// Created by shahwaz on 4/4/21.
//

#ifndef PACKAGE_TRACKING_BEND_CATEGORIZATION_H
#define PACKAGE_TRACKING_BEND_CATEGORIZATION_H

#include "point_cloud_utilities/pcl_utilities.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

class CloudCoordinates {
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

    CloudCoordinates(float x, float y, float z) : x(x), y(y), z(z) {}
};


class BendDetection {

public:
    BendDetection();

    ros::NodeHandle nh_;

    ros::Subscriber edge_points_sub_;
    std::string frame_id_;
    PointCloudT::Ptr cloud_scene_ptr_;
    float set_straight_length_{};
    float get_straight_length_{};
    std::string bend_check;

    void edgePointCb(const sensor_msgs::PointCloud2 &cloud_scene);

    void calculateEdgeLength();

    static std::vector<CloudCoordinates> edgePointProcessing(PointCloudT &cloudIn);

    static float straightEdge(std::vector<CloudCoordinates> &cloudCoordinates);

    float bendEdge(std::vector<CloudCoordinates> &cloudCoordinates) const;

};

#endif //PACKAGE_TRACKING_BEND_CATEGORIZATION_H
