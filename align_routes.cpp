#include <pcl/kdtree/kdtree_flann.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_set>

#include "utility.hpp"

namespace {

/// Match YAML::DoublePrecision(n) for values we add by hand (emitter precision cannot target a single node).
double round_decimal(double v, int n) {
    const double p = std::pow(10.0, n);
    return std::round(v * p) / p;
}

/// Collect all point ids from a YAML Points sequence (skips entries without id).
std::unordered_set<int> collect_point_ids(const YAML::Node& points) {
    std::unordered_set<int> ids;
    if (!points.IsDefined() || !points.IsSequence()) {
        return ids;
    }
    for (const auto& item : points) {
        if (!item["id"].IsDefined()) {
            continue;
        }
        ids.insert(item["id"].as<int>());
    }
    return ids;
}

} // namespace

const std::string base_map_name = "Maps/zhenhai-xun-2-3";
const std::string aligned_map_name = "Maps/zhenhai-road-6";
const std::string final_map_name = "Maps/zhenhai-xun-2-road-3";

// routes.yaml中的pose没有保存z坐标，需要从trajectory.pcd找到最近关键帧的z坐标
int main() {
    const std::string aligned_routes_file = aligned_map_name + "/routes.yaml";
    const std::string base_routes_file = base_map_name + "/routes.yaml";
    PointCloudPtr aligned_trajectory(new pcl::PointCloud<PointType>);
    loadPCDFile<PointType>(aligned_map_name + "/trajectory.pcd", aligned_trajectory);
    // 将LeGO的3d地图y轴（也就是ROS下的z轴）设置为0转换为2d地图
    // PointCloudPtr aligned_map_2d(new pcl::PointCloud<PointType>);
    // aligned_map_2d->resize(aligned_map->points.size());
    // for (size_t i = 0; i < aligned_map->points.size(); ++i) {
    //    const auto& psrc = aligned_map->points[i];
    //    auto& pdst = aligned_map_2d->points[i];
    //    pdst.x = psrc.x;
    //    pdst.y = 0; // LeGO格式地图y轴是ROS下的z轴
    //    pdst.z = psrc.z;
    // }
    // savePCDFile<PointType>(aligned_map_name + "/2d.pcd", aligned_map_2d);

    // T to base from aligned
    auto T_ba = loadTransformFromTxt<float>(final_map_name + "/R_t_final.txt");
    auto R_ba = T_ba.block<3, 3>(0, 0);
    auto t_ba = T_ba.block<3, 1>(0, 3);
    pcl::KdTreeFLANN<PointType> aligned_trajectory_kdtree(new pcl::KdTreeFLANN<PointType>);
    aligned_trajectory_kdtree.setInputCloud(aligned_trajectory);
    std::vector<int> point_indices;
    std::vector<float> point_distances;
    YAML::Node base_routes_root;
    try {
        base_routes_root = YAML::LoadFile(base_routes_file);
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading " << base_routes_file << ": " << e.what() << std::endl;
        return -1;
    }
    YAML::Node base_routes_points = base_routes_root["Points"];
    YAML::Node base_routes_edges = base_routes_root["Edges"];
    std::unordered_set<int> base_point_ids = collect_point_ids(base_routes_points);
    // {
    //     const std::string dump_path = "routes_dump.yaml";
    //     std::ofstream out(dump_path);
    //     if (!out) {
    //         std::cerr << "Error opening for write: " << dump_path << std::endl;
    //         return -1;
    //     }
    //     out << base_routes_root;
    //     std::cout << "Wrote " << dump_path << std::endl;
    //     return 0;
    // }

    try {
        YAML::Node aligned_routes_root = YAML::LoadFile(aligned_routes_file);
        YAML::Node aligned_routes_points = aligned_routes_root["Points"];
        YAML::Node aligned_routes_edges = aligned_routes_root["Edges"];
        // 把aligned_map的points坐标系变换到base_map的坐标系
        for (const auto& p : aligned_routes_points) {
            if (!p["id"].IsDefined()) {
                std::cerr << "routes: aligned point missing id\n";
                return -1;
            }
            const int point_id = p["id"].as<int>();
            if (base_point_ids.count(point_id) != 0) {
                std::cerr << "routes: duplicate point id " << point_id << " (already in base or repeated in aligned)\n";
                return -1;
            }
            auto pose = p["pose"].as<std::vector<double>>();
            PointType pose_fixed;
            pose_fixed.z = pose[0];
            pose_fixed.x = pose[1];
            pose_fixed.y = 0.0;
            point_indices.clear();
            point_distances.clear();
            int found = aligned_trajectory_kdtree.nearestKSearch(pose_fixed, 1, point_indices, point_distances);
            if (found > 0) {
                // point_distances is squared distance (PCL behavior), so take
                // sqrt for real distance.
                const auto index = point_indices[0];
                const auto& hit = aligned_trajectory->points[index];
                std::cout << "(" << pose_fixed.z << ", " << pose_fixed.x << ")"
                          << " dist=" << std::sqrt(point_distances[0]) << " hit=(" << hit.x << " " << hit.y << " "
                          << hit.z << ")" << std::endl;
                pose_fixed.y = hit.y;
            } else {
                std::cerr << "No points!" << std::endl;
            }
            Eigen::Vector3f pose_fixed_vec(pose_fixed.x, pose_fixed.y, pose_fixed.z);
            Eigen::Vector3f pose_transformed = R_ba * pose_fixed_vec + t_ba;
            pose_fixed.x = pose_transformed.x();
            pose_fixed.y = pose_transformed.y();
            pose_fixed.z = pose_transformed.z();
            std::cout << "Transformed pose: (" << pose_fixed.x << ", " << pose_fixed.y << ", " << pose_fixed.z << ")"
                      << std::endl;
            YAML::Node entry;
            entry["id"] = p["id"];
            entry["type"] = p["type"];
            entry["delay_time"] = p["delay_time"];
            YAML::Node pose_seq(YAML::NodeType::Sequence);
            pose_seq.SetStyle(YAML::EmitterStyle::Flow);
            constexpr int kPoseDecimals = 4;
            pose_seq.push_back(round_decimal(static_cast<double>(pose_fixed.z), kPoseDecimals));
            pose_seq.push_back(round_decimal(static_cast<double>(pose_fixed.x), kPoseDecimals));
            pose_seq.push_back(round_decimal(pose[2], kPoseDecimals)); // yaw 也应变换，此处未做
            entry["pose"] = pose_seq;
            entry["zone"] = p["zone"];
            entry["no_rotation"] = p["no_rotation"];
            entry["obstacle_detection"] = p["obstacle_detection"];
            entry["description"] = p["description"];
            base_routes_points.push_back(entry);
            base_point_ids.insert(point_id);
        }
        // Edges不涉及具体坐标，直接拷贝到base_routes_edges
        for (const auto& e : aligned_routes_edges) {
            base_routes_edges.push_back(e);
        }
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading routes file: " << e.what() << std::endl;
        return -1;
    }

    /// Write base_routes_root to final_map_name/routes.yaml
    {
        const std::string dump_path = final_map_name + "/routes.yaml";
        std::ofstream out(dump_path);
        if (!out) {
            std::cerr << "Error opening for write: " << dump_path << std::endl;
            return -1;
        }
        out << base_routes_root;
        std::cout << "Wrote " << dump_path << std::endl;
    }

    /// Append aligned arm_points.yaml entries after base (duplicate top-level key is an error).
    const std::string base_arm_points_file = base_map_name + "/arm_points.yaml";
    const std::string aligned_arm_points_file = aligned_map_name + "/arm_points.yaml";
    YAML::Node base_arm_points_root;
    try {
        base_arm_points_root = YAML::LoadFile(base_arm_points_file);
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading " << base_arm_points_file << ": " << e.what() << std::endl;
        return -1;
    }
    YAML::Node aligned_arm_points_root;
    try {
        aligned_arm_points_root = YAML::LoadFile(aligned_arm_points_file);
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading " << aligned_arm_points_file << ": " << e.what() << std::endl;
        return -1;
    }
    if (!base_arm_points_root.IsMap() || !aligned_arm_points_root.IsMap()) {
        std::cerr << "arm_points.yaml root must be a mapping\n";
        return -1;
    }
    std::unordered_set<std::string> arm_point_keys;
    for (const auto& kv : base_arm_points_root) {
        arm_point_keys.insert(kv.first.as<std::string>());
    }
    for (const auto& kv : aligned_arm_points_root) {
        const std::string key = kv.first.as<std::string>();
        if (key == "start_point") continue;
        if (arm_point_keys.count(key) != 0) {
            std::cerr << "arm_points: duplicate top-level key \"" << key << "\" (already in base)\n";
            return -1;
        }
        base_arm_points_root[key] = YAML::Clone(kv.second);
        arm_point_keys.insert(key);
    }
    {
        const std::string arm_out = final_map_name + "/arm_points.yaml";
        std::ofstream out(arm_out);
        if (!out) {
            std::cerr << "Error opening for write: " << arm_out << std::endl;
            return -1;
        }
        out << base_arm_points_root;
        std::cout << "Wrote " << arm_out << std::endl;
    }
    return 0;
}
