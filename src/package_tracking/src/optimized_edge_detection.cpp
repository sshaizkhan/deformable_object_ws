//
// Created by shahwaz on 3/5/21.
//

#include "package_tracking/optimized_edge_detection.h"

EdgeTracking::EdgeTracking() {

    package_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2>("/edge_cloud", 1);
    package_marker_publisher_ = node_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    camera_point_cloud_sub_ = node_.subscribe("two/depth/color/points", 1, &EdgeTracking::pointCloudCb, this);
    camera_scene_cloud_ptr_ = PointCloudT::Ptr(new PointCloudT);
    box_filter_cloud_ptr_ = PointCloudT::Ptr(new PointCloudT);
}

void EdgeTracking::pointCloudCb(const sensor_msgs::PointCloud2 &cloud_scene) {
    frame_id_ = cloud_scene.header.frame_id;
    pcl::fromROSMsg(cloud_scene, *camera_scene_cloud_ptr_);
    std::cout << "Number of points from camera: " << camera_scene_cloud_ptr_->points.size() << std::endl;
    optimizedTrackEdge();
}

void EdgeTracking::optimizedTrackEdge() {
    boxFilter();
//    PointCloudT::Ptr euclidean_point_cloud_  (new PointCloudT);
    PointCloudT::Ptr final_cloud_(new PointCloudT);
    const vector<Coordinate> &point_cloud_vector = cloudProcessing(*box_filter_cloud_ptr_);
    const vector<EdgePoint> &obj_edges = finalEdgeTracking(point_cloud_vector, z_tolerance);
    *final_cloud_ = buildCloud(obj_edges);
//    *euclidean_point_cloud_ = PCLUtilities::euclideanClustering(final_cloud_, ec_tolerance, minClusterSize, maxClusterSize);
    PCLUtilities::publishPCLToRviz(*final_cloud_, package_cloud_publisher_, frame_id_);
    PCLUtilities::rvizMarkerPublish(*final_cloud_, package_marker_publisher_, frame_id_, "line");
    PCLUtilities::savePointCloudToPLY(buildCloud(obj_edges), file_path_, "P1.ply");
    std::cout << "Publishing to RViz...." << std::endl;
}

void EdgeTracking::boxFilter() {
    // Using a CropBox filter to extract the region of interest from the camera scene.
    pcl::CropBox<pcl::PointXYZ> box_filter;
    std::cout << "Input Point Cloud has: " << camera_scene_cloud_ptr_->points.size() << std::endl;
    box_filter.setInputCloud(camera_scene_cloud_ptr_->makeShared());
    box_filter.setMin(Eigen::Vector4f(camera_box_limits_[0], camera_box_limits_[2], camera_box_limits_[4], 1.0));
    box_filter.setMax(Eigen::Vector4f(camera_box_limits_[1], camera_box_limits_[3], camera_box_limits_[5], 1.0));
    box_filter.filter(*box_filter_cloud_ptr_);
    std::cout << "The box-filtered point cloud has: " << box_filter_cloud_ptr_->points.size() << std::endl;

}

std::vector<Coordinate> EdgeTracking::cloudProcessing(PointCloudT &cloudIn) {
    std::vector<Coordinate> pcl_vector_;
    for (auto &cloud : cloudIn) {
        pcl_vector_.emplace_back(PCLUtilities::round(cloud.x, 0.001), cloud.y, PCLUtilities::round(cloud.z, 0.001));
    }
    std::cout << "PointCloud after storing in vectors has : " << pcl_vector_.size()
              << " data points" << std::endl;
    return pcl_vector_;
}

std::vector<EdgePoint> EdgeTracking::finalEdgeTracking(const std::vector<Coordinate> &coordinates, float &z_tolerance) {
//    std::unordered_map<float, std::vector<std::pair<float,float>>> x_coordinate_map_;
    std::unordered_map<float, std::vector<std::pair<float, float>>> x_coordinate_map_;
    std::cout << "input coordinate has size : " << coordinates.size() << std::endl;

    for (auto coordinate : coordinates) {
        x_coordinate_map_[coordinate.getX()].push_back(std::make_pair(coordinate.getZ(), coordinate.getY()));
    }
    std::cout << "x_coordinate map has size : " << x_coordinate_map_.size() << std::endl;

     std::vector<EdgePoint>result;
    for (auto& it : x_coordinate_map_)
    {
        sort(it.second.begin(), it.second.end());
        EdgePoint edgePoint(it.first, it.second[0].first, it.second[0].second);
        result.push_back(edgePoint);
    }
    return result;
}

pcl::PointCloud<pcl::PointXYZ> EdgeTracking::buildCloud(const std::vector<EdgePoint> &edgePoints) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr final_point_cloud_(new pcl::PointCloud<pcl::PointXYZ>);
    final_point_cloud_->width = edgePoints.size();
    final_point_cloud_->height = 1;
    final_point_cloud_->points.clear();
    final_point_cloud_->points.resize(final_point_cloud_->width * final_point_cloud_->height);

    for (std::size_t i = 0; i < edgePoints.size(); i++) {
        final_point_cloud_->points[i].x = edgePoints[i].getXCoordinate();
        final_point_cloud_->points[i].z = edgePoints[i].getMinZCoordinate();
        final_point_cloud_->points[i].y = edgePoints[i].getCorrespondingY();
    }
    std::cout << "PointCloud after creating from vectors has : " << final_point_cloud_->points.size()
              << " data points" << std::endl;
    return *final_point_cloud_;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "optimized_edge_tracking");
    EdgeTracking edgeObj;
    ros::NodeHandle pnh("~");
    pnh.param("cam_bounding_box", edgeObj.camera_box_limits_, std::vector<double>());
    pnh.param("z_tolerance", edgeObj.z_tolerance, float());
    ros::Rate loop_rate(30);

    while (ros::ok()) {
        ros::spin();
    }
    return 0;
}
