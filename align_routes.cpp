#include <pcl/kdtree/kdtree_flann.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_set>

#include "utility.hpp"
#include "Pose.hpp"

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

bool load_yaml_file(const std::string& path, YAML::Node& out) {
    try {
        out = YAML::LoadFile(path);
        return true;
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading " << path << ": " << e.what() << std::endl;
        return false;
    }
}

bool write_yaml_file(const std::string& path, const YAML::Node& node) {
    std::ofstream out(path);
    if (!out) {
        std::cerr << "Error opening for write: " << path << std::endl;
        return false;
    }
    out << node;
    std::cout << "Wrote " << path << std::endl;
    return true;
}

/// routes.yaml中的pose没有保存z坐标，需要从trajectory.pcd找到最近关键帧的z坐标，
/// 然后变换到base坐标系，返回构造好的YAML条目。
YAML::Node transform_route_point(const YAML::Node& p,
                                 const pcl::KdTreeFLANN<PointType>& kdtree,
                                 const PointCloudPtr& trajectory,
                                 const std::map<int, Pose>& poses,
                                 const Eigen::Matrix3f& R_ba,
                                 const Eigen::Vector3f& t_ba) {
    auto tpose = p["pose"].as<std::vector<double>>();
    PointType point_fixed;
    point_fixed.z = tpose[0];
    point_fixed.x = tpose[1];
    point_fixed.y = 0.0;
    Eigen::Vector3f point_rpy(0.0, tpose[2], 0.0);
    std::vector<int> point_indices;
    std::vector<float> point_distances;
    int found = kdtree.nearestKSearch(point_fixed, 1, point_indices, point_distances);
    if (found > 0) {
        const auto index = point_indices[0];
        const auto& hit = trajectory->points[index];
        std::cout << "(" << point_fixed.z << ", " << point_fixed.x << ")"
                  << " dist=" << std::sqrt(point_distances[0]) << " hit=(" << hit.x << " " << hit.y << " " << hit.z
                  << ")" << std::endl;
        point_fixed.y = hit.y;
        int pose_id = static_cast<int>(hit.intensity);
        auto nearest_pose = poses.at(pose_id);
        point_rpy[0] = nearest_pose.rpy[0];
        point_rpy[2] = nearest_pose.rpy[2];
    } else {
        std::cerr << "No points!" << std::endl;
    }

    Eigen::Vector3f point_fixed_vec(point_fixed.x, point_fixed.y, point_fixed.z);
    Eigen::Vector3f point_transformed = R_ba * point_fixed_vec + t_ba;
    point_fixed.x = point_transformed.x();
    point_fixed.y = point_transformed.y();
    point_fixed.z = point_transformed.z();
    std::cout << "Transformed pose: (" << point_fixed.x << ", " << point_fixed.y << ", " << point_fixed.z << ")"
              << std::endl;
    Eigen::Matrix3f R_pose = (Eigen::AngleAxisf(point_rpy[2], Eigen::Vector3f::UnitZ()) *
                              Eigen::AngleAxisf(point_rpy[1], Eigen::Vector3f::UnitY()) *
                              Eigen::AngleAxisf(point_rpy[0], Eigen::Vector3f::UnitX()))
                                 .toRotationMatrix();
    Eigen::Vector3f point_rpy_transformed = R_pose * point_rpy;
    YAML::Node entry;
    entry["id"] = p["id"];
    entry["type"] = p["type"];
    entry["delay_time"] = p["delay_time"];
    YAML::Node pose_seq(YAML::NodeType::Sequence);
    pose_seq.SetStyle(YAML::EmitterStyle::Flow);
    constexpr int kPoseDecimals = 4;
    pose_seq.push_back(round_decimal(static_cast<double>(point_fixed.z), kPoseDecimals));
    pose_seq.push_back(round_decimal(static_cast<double>(point_fixed.x), kPoseDecimals));
    pose_seq.push_back(round_decimal(point_rpy_transformed[1], kPoseDecimals));  // yaw 也应变换，此处未做
    entry["pose"] = pose_seq;
    entry["zone"] = p["zone"];
    entry["no_rotation"] = p["no_rotation"];
    entry["obstacle_detection"] = p["obstacle_detection"];
    entry["description"] = p["description"];
    return entry;
}

/// 把aligned_map的routes变换并合并到base_map，输出到final_map。
int merge_routes(const std::string& base_map_name,
                 const std::string& aligned_map_name,
                 const std::string& final_map_name) {
    PointCloudPtr aligned_trajectory(new pcl::PointCloud<PointType>);
    loadPCDFile<PointType>(aligned_map_name + "/trajectory.pcd", aligned_trajectory);
    std::map<int, Pose> aligned_poses;
    loadPosesFromFile(aligned_map_name + "/pose.txt", aligned_poses);
    // T to base from aligned
    auto T_ba = loadTransformFromTxt<float>(final_map_name + "/R_t_final.txt");
    Eigen::Matrix3f R_ba = T_ba.block<3, 3>(0, 0);
    Eigen::Vector3f t_ba = T_ba.block<3, 1>(0, 3);

    pcl::KdTreeFLANN<PointType> aligned_trajectory_kdtree(new pcl::KdTreeFLANN<PointType>);
    aligned_trajectory_kdtree.setInputCloud(aligned_trajectory);

    YAML::Node base_routes_root;
    if (!load_yaml_file(base_map_name + "/routes.yaml", base_routes_root)) {
        return -1;
    }
    YAML::Node base_routes_points = base_routes_root["Points"];
    YAML::Node base_routes_edges = base_routes_root["Edges"];
    std::unordered_set<int> base_point_ids = collect_point_ids(base_routes_points);

    try {
        YAML::Node aligned_routes_root = YAML::LoadFile(aligned_map_name + "/routes.yaml");
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
            base_routes_points.push_back(
                transform_route_point(p, aligned_trajectory_kdtree, aligned_trajectory, aligned_poses, R_ba, t_ba));
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

    if (!write_yaml_file(final_map_name + "/routes.yaml", base_routes_root)) {
        return -1;
    }
    return 0;
}

/// 把aligned_map的arm_points合并到base_map（跳过start_point，检查重复key），输出到final_map。
int merge_arm_points(const std::string& base_map_name,
                     const std::string& aligned_map_name,
                     const std::string& final_map_name) {
    YAML::Node base_arm_points_root;
    if (!load_yaml_file(base_map_name + "/arm_points.yaml", base_arm_points_root)) {
        return -1;
    }
    YAML::Node aligned_arm_points_root;
    if (!load_yaml_file(aligned_map_name + "/arm_points.yaml", aligned_arm_points_root)) {
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

    if (!write_yaml_file(final_map_name + "/arm_points.yaml", base_arm_points_root)) {
        return -1;
    }
    return 0;
}

} // namespace

const char* config_file = "align_routes.yaml";
std::string base_map_name = "Maps/zhenhai-xun-2-3";
std::string aligned_map_name = "Maps/zhenhai-road-6";
std::string final_map_name = "Maps/zhenhai-xun-2-road-3";

int main() {
    try {
        YAML::Node cfg = YAML::LoadFile(config_file);
        base_map_name = cfg["base_map_dir"].as<std::string>("Maps/zhenhai-xun-2-3");
        aligned_map_name = cfg["align_map_dir"].as<std::string>("Maps/zhenhai-road-6");
        final_map_name = cfg["final_map_dir"].as<std::string>("Maps/zhenhai-xun-2-road-3");
    } catch (const std::exception& e) {
        std::cerr << "Error loading " << config_file << e.what() << std::endl;
        return -1;
    }
    if (merge_routes(base_map_name, aligned_map_name, final_map_name) != 0) {
        return -1;
    }
    if (merge_arm_points(base_map_name, aligned_map_name, final_map_name) != 0) {
        return -1;
    }
    return 0;
}
