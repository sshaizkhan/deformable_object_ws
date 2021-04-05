//
// Created by shahwazk@usc.edu on 4/4/21.
//

#include "hinge_detection/bend_categorization.h"


BendDetection::BendDetection() {

    edge_points_sub_  = nh_.subscribe("/edge_cloud", 1, &BendDetection::edgePointCb, this);
    cloud_scene_ptr_ = PointCloudT::Ptr (new PointCloudT);
}

void BendDetection::edgePointCb(const sensor_msgs::PointCloud2 &cloud_scene) {

    frame_id_ = cloud_scene.header.frame_id;
    pcl::fromROSMsg(cloud_scene, *cloud_scene_ptr_);
    std::cout << "Number of points from camera: " << cloud_scene_ptr_->points.size() << std::endl;
    calculateEdgeLength();
    edge_points_sub_.shutdown();
}

void BendDetection::calculateEdgeLength() {
    std::vector<CloudCoordinates> final_cloud_coordinates;
    final_cloud_coordinates = edgePointProcessing(*cloud_scene_ptr_);

    if (bend_check == "straight") {
        set_straight_length_ = straightEdge(final_cloud_coordinates);
        nh_.setParam("package_length", set_straight_length_);

        std::cout << "===================================================================" << std::endl;
        std::cout << "|                                                                 | " << std::endl;
        std::cout << "                          Edge length in m: " << set_straight_length_ << std::endl;
        std::cout << "|                                                                 |" << std::endl;
        std::cout << "===================================================================" << std::endl;
    }

    else if (bend_check == "bend")
    {
        nh_.getParam("package_length", get_straight_length_);
        bendEdge(final_cloud_coordinates);
    }

}

std::vector<CloudCoordinates> BendDetection::edgePointProcessing(PointCloudT& cloudIn) const {
    std::vector<CloudCoordinates> cloud_coordinates;
    for (auto &cloud: cloudIn)
    {
        cloud_coordinates.emplace_back(cloud.x, cloud.y, 0);
    }
    std::cout << "PointCloud after storing in vectors has : " << cloud_coordinates.size()
              << " data points" << std::endl;
    return cloud_coordinates;

}

float BendDetection::straightEdge(vector<CloudCoordinates> &cloudCoordinates) const {
    std::vector<float> x_coordinate;
    for (auto &cloudCoordinate : cloudCoordinates)
    {
        x_coordinate.emplace_back(cloudCoordinate.getX());
    }
    sort(x_coordinate.begin(), x_coordinate.end());
    float straight_edge_length = abs(x_coordinate[0] - x_coordinate[x_coordinate.size() - 1]);
    return straight_edge_length;
}

float BendDetection::bendEdge(vector<CloudCoordinates> &cloudCoordinates) const {
    std::vector<float> x_coordinate;
    std::vector<float> y_coordinate;

    for (auto &cloudCoordinate: cloudCoordinates)
    {
        x_coordinate.emplace_back(cloudCoordinate.getX());
        y_coordinate.emplace_back(cloudCoordinate.getY());
    }
    sort(x_coordinate.begin(), x_coordinate.end());
    sort(y_coordinate.begin(), y_coordinate.end());

    //// (P1) ===========================Package Edge==============================(P2) ////
    ////   (min) -                             (Q1)(min)                      (max) -
        ////      -                          |                            -
            ////      -                    (Y)                      -
                ////      -L1             Bend Line          L2 -
                    ////      -             |               -
                        ////      -         |           -
                            ////      -             -
                                ////      -  (Q2)(max)

    std::vector<float> P1;
    std::vector<float> P2;
    std::vector<float> Q1;
    std::vector<float> Q2;

//    Push back the min value in x-direction
    P1.push_back(x_coordinate[0]);

//    Push back the max value in x-direction
    P2.push_back(x_coordinate[x_coordinate.size() - 1]);


    for (auto &cloudCoordinate : cloudCoordinates)
    {
        if (cloudCoordinate.getX() == x_coordinate[0])
        {
            P1.push_back(cloudCoordinate.getY());
        }
        else if (cloudCoordinate.getX() == x_coordinate[x_coordinate.size() - 1])
        {
            P2.push_back(cloudCoordinate.getY());
        }
        else if (cloudCoordinate.getY() == y_coordinate[0])
        {
            Q1.push_back(cloudCoordinate.getX());
        }
        else if (cloudCoordinate.getY() == y_coordinate[y_coordinate.size() - 1])
        {
            Q2.push_back(cloudCoordinate.getX());
        }
    }
//    Push back the min value in y-direction
    Q1.push_back(y_coordinate[0]);

//    Push back the max value in y-direction
    Q2.push_back(y_coordinate[y_coordinate.size() - 1]);


    std::cout << "Straight length: " << get_straight_length_ << std::endl;
    float P1_x = P1[0] - Q2[0];
    float P1_y = P1[1] - Q2[1];

    float L1;
    L1 = pow(P1_x, 2) + pow(P1_y, 2);
    L1 = sqrt(L1);
    std::cout << "L1: " << L1 << std::endl;

    float P2_x = P2[0] - Q2[0];
    float P2_y = P2[1] - Q2[1];

    float L2;
    L2 = pow(P2_x, 2) + pow(P2_y, 2);
    L2 = sqrt(L2);
    std::cout << "L2: " << L2 << std::endl;

    if (abs((L1 + L2) - get_straight_length_) < 0.01) {
        if (abs(P2[1] - Q2[1]) > 0.002)
        {
            std::cout << "===================================================================" << std::endl;
            std::cout << "|                                                                 | " << std::endl;
            std::cout << "                          NO BEND: "                                  << std::endl;
            std::cout << "|                                                                 |" << std::endl;
            std::cout << "===================================================================" << std::endl;
        }
        else
        {
            std::cout << "===================================================================" << std::endl;
            std::cout << "|                                                                 | " << std::endl;
            std::cout << "                          MID BEND: "                                 << std::endl;
            std::cout << "|                                                                 |" << std::endl;
            std::cout << "===================================================================" << std::endl;
        }
    }
    else
    {
        std::cout << "===================================================================" << std::endl;
        std::cout << "|                                                                 | " << std::endl;
        std::cout << "                          CORNER BEND: "                              << std::endl;
        std::cout << "|                                                                 |" << std::endl;
        std::cout << "===================================================================" << std::endl;
    }

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "bend_categorization");
    BendDetection bend;
    bend.bend_check = argv[1];

    while (ros::ok()){
        ros::spin();
    }
    return 0;
}



