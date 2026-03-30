#include "utility.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <iostream>
#include <cmath>

const std::string base_map_name = "Maps/zhenhai-xun-2-3"
const std::string aligned_map_name = "Maps/zhenhai-road-6";
const std::string final_map_name =  "Maps/zhenhai-xun-2-road-3"

int main() {
    PointCloudPtr aligned_map(new pcl::PointCloud<pcl::PointXYZI>);
    loadPCDFile<PointType>(aligned_map_name + "/trajectory.pcd", aligned_map);
    PointCloudPtr aligned_map_2d(new pcl::PointCloud<PointType>);
    aligned_map_2d->resize(aligned_map->points.size());
    for (size_t i = 0; i < aligned_map->points.size(); ++i) {
       const auto& psrc = aligned_map->points[i];
       auto& pdst = aligned_map_2d->points[i];
       pdst.x = psrc.x;
       pdst.y = 0; // LeGO格式地图y轴是ROS下的z轴
       pdst.z = psrc.z;
    }
    // savePCDFile<PointType>(aligned_map_name + "/2d.pcd", aligned_map_2d);
    pcl::KdTreeFLANN<PointType> aligned_map_kdtree(new pcl::KdTreeFLANN<PointType>);
    aligned_map_kdtree.setInputCloud(aligned_map_2d);
    std::vector<int> point_indices;
    std::vector<float> point_distances;
    double search_radius = 10.0;
    const std::string routes_file = aligned_map_name + "/routes.yaml";
    try {
        YAML::Node root = YAML::LoadFile(routes_file);
        YAML::Node points = root["Points"];
        for (const auto& p: points) {
            auto pose = p["pose"].as<std::vector<double>>();
            PointType pose_fixed;
            pose_fixed.z = pose[0];
            pose_fixed.x = pose[1];
            pose_fixed.y = 0.0;
            point_indices.clear();
            point_distances.clear();
            int found = aligned_map_kdtree.nearestKSearch(
                pose_fixed, 1, point_indices, point_distances);
            if (found > 0) {
                // point_distances is squared distance (PCL behavior), so take sqrt for real distance.
                const auto index = point_indices[0];
                const auto& hit = aligned_map->points[index];
                std::cout << "Nearest 1 point to (" << pose_fixed.z << ", "
                          << pose_fixed.x
                          << ") dist=" << std::sqrt(point_distances[0])
                          << " hit=(" << hit.x << " " << hit.y << " " << hit.z
                          << ")" << std::endl;
                pose_fixed.y = hit.y;

            } else {
                std::cout << "No points!" << std::endl;
            }
        }
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading routes file: " << e.what() << std::endl;
        return -1;
    }

}
