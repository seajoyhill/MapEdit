#include "utility.hpp"
#include "Pose.hpp"
#include "MapInfo.h"
#include <pcl/filters/crop_box.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <map>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

const char* config_file = "align_map.yaml";
double thre_z_min = -0.2;
double thre_z_max = 2.0;
std::string base_map_name = "Maps/22F";
std::string aligned_map_name = "Maps/22F-ext";
std::string final_map_name = "Maps/22F-final";
Eigen::Vector3f align_to_base_t = Eigen::Vector3f::Zero();
Eigen::Vector3f align_to_base_rpy_deg = Eigen::Vector3f::Zero();
Eigen::Vector3f align_to_base_rpy_rad = Eigen::Vector3f::Zero();
double ndt_transformation_epsilon = 0.001;
int ndt_maximum_iterations = 50;
bool base_match_roi_enable = false;
bool base_match_use_ros_coord = false;
double base_match_roi_x_min = 0.0;
double base_match_roi_y_min = 0.0;
double base_match_roi_z_min = 0.0;
double base_match_roi_x_max = 0.0;
double base_match_roi_y_max = 0.0;
double base_match_roi_z_max = 0.0;

void printParams() {
    std::cout << "==================== " << "MapEdit's params" <<  " ===================="<< std::endl;
    std::cout << "base_map_name: " << base_map_name << std::endl;
    std::cout << "aligned_map_name: " << aligned_map_name << std::endl;
    std::cout << "final_map_name: " << final_map_name << std::endl;
    std::cout << "align_to_base_t: [" << align_to_base_t.transpose() << "]" << std::endl;
    std::cout << "align_to_base_rpy_deg: [" << align_to_base_rpy_deg.transpose() << "]" << std::endl;
    std::cout << "thre_z_min: " << thre_z_min << std::endl;
    std::cout << "thre_z_max: " << thre_z_max << std::endl;
    std::cout << "ndt_transformation_epsilon: " << ndt_transformation_epsilon << std::endl;
    std::cout << "ndt_maximum_iterations: " << ndt_maximum_iterations << std::endl;
    std::cout << "base_match_roi_enable: " << (base_match_roi_enable ? "true" : "false") << std::endl;
    if (base_match_roi_enable) {
        std::cout << "base_match_roi (ROS horizontal XY, same axes as align_to_base_t): [" << base_match_roi_x_min
                  << ", " << base_match_roi_y_min << ", " << base_match_roi_x_max << ", " << base_match_roi_y_max
                  << "]" << std::endl;
    }
    std::cout << "==================== " << "MapEdit's params" <<  " ===================="<< std::endl;
}

int main(int argc, char **argv)
{
    // =================== 1.读取配置参数 ===================
    try {
        YAML::Node cfg = YAML::LoadFile(config_file);
        base_map_name = cfg["base_map_dir"].as<std::string>("Maps/22F");
        aligned_map_name = cfg["align_map_dir"].as<std::string>("Maps/22F-ext");
        final_map_name = cfg["final_map_dir"].as<std::string>("Maps/22F-final");
        thre_z_min = cfg["thre_z_min"].as<double>(-0.2);
        thre_z_max = cfg["thre_z_max"].as<double>(2.0);
        std::vector<double> t_vec = cfg["align_to_base_t"].as<std::vector<double>>();
        if (t_vec.size() == 3) {
            align_to_base_t = Eigen::Vector3f(t_vec[0], t_vec[1], t_vec[2]);
        } else {
            std::cerr << "align_to_base_t size is not 3!" << std::endl;
            return -1;
        }
        std::vector<double> rpy_vec = cfg["align_to_base_rpy_deg"].as<std::vector<double>>();
        if (rpy_vec.size() == 3) {
            align_to_base_rpy_deg = Eigen::Vector3f(rpy_vec[0], rpy_vec[1], rpy_vec[2]);
            align_to_base_rpy_rad = Eigen::Vector3f(rpy_vec[0] * M_PI / 180.0, rpy_vec[1] * M_PI / 180.0, rpy_vec[2] * M_PI / 180.0);
        } else {
            std::cerr << "align_to_base_rpy_deg size is not 3!" << std::endl;
            return -1;
        }
        ndt_transformation_epsilon = cfg["ndt_transformation_epsilon"].as<double>(0.001);
        ndt_maximum_iterations = cfg["ndt_maximum_iterations"].as<int>(500);
        if (cfg["base_match_roi_enable"]) {
            base_match_roi_enable = cfg["base_match_roi_enable"].as<bool>();
        }
        if (base_match_roi_enable) {
            std::vector<double> roi = cfg["base_match_roi"].as<std::vector<double>>();
            if (roi.size() != 6) {
                std::cerr << "base_match_roi must have 6 values: [x_min, y_min, z_min, x_max, y_max, z_max] in ROS frame"
                          << std::endl;
                return -1;
            }
            base_match_roi_x_min = roi[0];
            base_match_roi_y_min = roi[1];
            base_match_roi_z_min = roi[2];
            base_match_roi_x_max = roi[3];
            base_match_roi_y_max = roi[4];
            base_match_roi_z_max = roi[5];
            if (!(base_match_roi_x_min < base_match_roi_x_max && base_match_roi_y_min < base_match_roi_y_max && base_match_roi_z_min < base_match_roi_z_max)) {
                std::cerr << "base_match_roi: require x_min < x_max and y_min < y_max and z_min < z_max" << std::endl;
                return -1;
            }
            base_match_use_ros_coord = cfg["base_match_use_ros_coord"].as<bool>(false);
        }
    } catch (const std::exception& e) {
        std::cerr << "Error loading " << config_file << e.what() << std::endl;
        return -1;
    }
    printParams();

    // NOTICE: Convert from ROS coordinate to LeGO-LOAM coordinate
    Eigen::Vector3f align_to_base_t_lego;
    align_to_base_t_lego.x() = align_to_base_t.y();
    align_to_base_t_lego.y() = align_to_base_t.z();
    align_to_base_t_lego.z() = align_to_base_t.x();
    Eigen::Vector3f align_to_base_rpy_lego;
    align_to_base_rpy_lego.x() = align_to_base_rpy_rad.y();
    align_to_base_rpy_lego.y() = align_to_base_rpy_rad.z();
    align_to_base_rpy_lego.z() = align_to_base_rpy_rad.x();

    // =================== 2.加载点云并初步对齐 ===================
    PointCloudPtr base_map(new pcl::PointCloud<pcl::PointXYZI>);
    loadPCDFile<PointType>(base_map_name + "/CornerMap.pcd", base_map);
    // convertToROSCoordinate<PointType>(base_map);
    // savePCDFile<PointType>(base_map_name + "/GlobalMap_ros.pcd", base_map); // 调试

    PointCloudPtr aligned_map(new pcl::PointCloud<pcl::PointXYZI>);
    loadPCDFile<PointType>(aligned_map_name + "/CornerMap.pcd", aligned_map);
    // convertToROSCoordinate<PointType>(aligned_map);
    // savePCDFile<PointType>(aligned_map_name + "/GlobalMap_ros.pcd", aligned_map); // 调试

    auto align_to_base_R_lego = (Eigen::AngleAxisf(align_to_base_rpy_lego[2], Eigen::Vector3f::UnitZ()) *
                                 Eigen::AngleAxisf(align_to_base_rpy_lego[1], Eigen::Vector3f::UnitY()) *
                                 Eigen::AngleAxisf(align_to_base_rpy_lego[0], Eigen::Vector3f::UnitX())).toRotationMatrix();

    PointCloudPtr aligned_map_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    aligned_map_transformed->resize(aligned_map->points.size());
    applyTransform<PointType>(aligned_map, aligned_map_transformed, align_to_base_R_lego, align_to_base_t_lego);
    savePCDFile<PointType>(aligned_map_name + "/CornerMap_transformed.pcd", aligned_map_transformed);

    // =================== 3.NDT精细对齐 ===================
    PointCloudPtr base_map_for_ndt = base_map;
    if (base_match_roi_enable) {
        PointCloudPtr base_cropped(new pcl::PointCloud<PointType>);
        Eigen::Vector4f min_lego, max_lego;
        if (base_match_use_ros_coord) {
            min_lego << base_match_roi_y_min, base_match_roi_z_min, base_match_roi_x_min, 1.0f;
            max_lego << base_match_roi_y_max, base_match_roi_z_max, base_match_roi_x_max, 1.0f;
        } else {
            min_lego << base_match_roi_x_min, base_match_roi_y_min, base_match_roi_z_min, 1.0f;
            max_lego << base_match_roi_x_max, base_match_roi_y_max, base_match_roi_z_max, 1.0f;
        }
        pcl::CropBox<PointType> crop;
        crop.setInputCloud(base_map);
        crop.setMin(min_lego);
        crop.setMax(max_lego);
        crop.setNegative(false);
        crop.filter(*base_cropped);
        std::cout << "NDT base map: cropped to ROS ROI (LeGO crop box min/max): min [" << min_lego.head<3>().transpose()
                  << "] max [" << max_lego.head<3>().transpose() << "]" << std::endl;
        std::cout << "NDT base map: " << base_cropped->points.size() << " points (full base: "
                  << base_map->points.size() << ")" << std::endl;
        if (base_cropped->points.empty()) {
            std::cerr << "base_match_roi produced empty point cloud; check ROI (ROS) and thre_z_min/thre_z_max"
                      << std::endl;
            return -1;
        }
        savePCDFile<PointType>(final_map_name + "/NdtMatchRoi_base_lego.pcd", base_cropped);
        base_map_for_ndt = base_cropped;
    }

    PointCloudPtr final_cloud(new pcl::PointCloud<PointType>);
    pcl::NormalDistributionsTransform<PointType, PointType> ndt;
    ndt.setTransformationEpsilon(ndt_transformation_epsilon);
    ndt.setMaximumIterations(ndt_maximum_iterations);
    ndt.setInputSource(aligned_map_transformed);
    ndt.setInputTarget(base_map_for_ndt);
    ndt.align(*final_cloud);
    auto finalTransformation = ndt.getFinalTransformation(); // NOTICE: 这是我们最终需要的变换矩阵
    std::cout << "==================== " << "NDT alignment result" <<  " ===================="<< std::endl;
    std::cout << "NDT alignment completed in " << ndt.getFinalNumIteration() << " iterations." << std::endl;
    std::cout << "Final transformation epsilon: " << ndt.getTransformationEpsilon() << std::endl;
    std::cout << "Final finalTransformation matrix:\n" << finalTransformation << std::endl;
    Eigen::Matrix3f R_ndt = finalTransformation.block<3, 3>(0, 0);
    Eigen::Vector3f t_ndt = finalTransformation.block<3, 1>(0, 3);
    // 将初步对齐的变换和NDT的变换结合，得到最终的变换
    Eigen::Matrix3f R_final = R_ndt * align_to_base_R_lego;
    Eigen::Vector3f t_final = R_ndt * align_to_base_t_lego + t_ndt;

    // Save final transform (matrix form): 4x4 homogeneous matrix [R t; 0 0 0 1]
    Eigen::Matrix4f T_final = Eigen::Matrix4f::Identity();
    T_final.block<3, 3>(0, 0) = R_final;
    T_final.block<3, 1>(0, 3) = t_final;
    {
        const std::string out_file = final_map_name + "/R_t_final.txt";
        std::ofstream ofs(out_file);
        if (!ofs.is_open()) {
            std::cerr << "Failed to open output file: " << out_file << std::endl;
        } else {
            ofs << std::setprecision(12);
            ofs << T_final << "\n";
        }
    }

    // =================== 4.合并pose.txt和trajectory.pcd ===================
    std::map<int, Pose> base_poses;
    loadPosesFromFile(base_map_name + "/pose.txt", base_poses);
    std::map<int, Pose> aligned_poses;
    loadPosesFromFile(aligned_map_name + "/pose.txt", aligned_poses);

    int base_traj_max_idx = base_poses.rbegin()->first;
    std::cout << "Base Traj Max Idx: " << base_traj_max_idx << std::endl;

    PointCloudPtr base_trajectory(new pcl::PointCloud<pcl::PointXYZI>);
    loadPCDFile<PointType>(base_map_name + "/trajectory.pcd", base_trajectory); // TODO: 对于直接在修改后的地图上运行不需要再加载trajectory.pcd

    // 加载准备合并的地图的trajectory.pcd，修改它的位姿序号从base_map最大值开始递增
    PointCloudPtr aligned_trajectory(new pcl::PointCloud<pcl::PointXYZI>);
    loadPCDFile<PointType>(aligned_map_name + "/trajectory.pcd", aligned_trajectory);
    for (size_t i = 0; i < aligned_trajectory->points.size(); ++i) {
        auto &point = aligned_trajectory->points[i];
        int idx = static_cast<int>(point.intensity);
        point.intensity = idx + base_traj_max_idx + 1;
        Eigen::Vector3f pt(point.x, point.y, point.z);
        Eigen::Vector3f pt_transformed = R_final * pt + t_final;
        point.x = pt_transformed.x();
        point.y = pt_transformed.y();
        point.z = pt_transformed.z();
        auto &pose = aligned_poses.at(idx);
        pose.index = idx + base_traj_max_idx + 1;
        pose.position = align_to_base_R_lego * pose.position + align_to_base_t_lego;
        Eigen::Matrix3f R_pose = (Eigen::AngleAxisf(pose.rpy[2], Eigen::Vector3f::UnitZ()) *
                                  Eigen::AngleAxisf(pose.rpy[1], Eigen::Vector3f::UnitY()) *
                                  Eigen::AngleAxisf(pose.rpy[0], Eigen::Vector3f::UnitX()))
                                     .toRotationMatrix();
        R_pose = R_final * R_pose;
        auto zyx = R_pose.eulerAngles(2, 1, 0);
        pose.rpy.x() = zyx[2];
        pose.rpy.y() = zyx[1];
        pose.rpy.z() = zyx[0];
        // final_poses.emplace_back(aligned_poses.at(idx));
        // std::cout << "Aligned Traj Point " << i << ": " << point.x << " " << point.y << " " << point.z << " " << point.intensity << std::endl;
    }
    std::vector<Pose> final_poses;
    for (const auto& kv : base_poses) {
        if (kv.second.active) {
            final_poses.emplace_back(kv.second);
        }
    }
    for (auto& kv : aligned_poses) {
        final_poses.emplace_back(kv.second);
    }
    savePosesToFile(final_map_name + "/pose.txt", final_poses); // 1.保存融合后的pose.txt

    PointCloudPtr merged_trajectory(new pcl::PointCloud<pcl::PointXYZI>);
    *merged_trajectory = *base_trajectory + *aligned_trajectory;
    savePCDFile<PointType>(final_map_name + "/trajectory.pcd", merged_trajectory); // 2.保存融合后的trajectory.pcd

    // =================== 5.根据得到的变换合并地图 ===================
    // 加载CornerMap.pcd、SurfMap.pcd，应用R_final和t_final变换后与base_map的地图点云合并，最后合并成GlobalMap.pcd
    PointCloudPtr base_corner_map(new pcl::PointCloud<PointType>);
    loadPCDFile<PointType>(base_map_name + "/CornerMap.pcd", base_corner_map);
    PointCloudPtr aligned_corner_map(new pcl::PointCloud<PointType>);
    loadPCDFile<PointType>(aligned_map_name + "/CornerMap.pcd", aligned_corner_map);
    for (size_t i = 0; i < aligned_corner_map->points.size(); ++i) {
        auto& point = aligned_corner_map->points[i];
        int idx = static_cast<int>(point.intensity);
        point.intensity = idx + base_traj_max_idx + 1;
    }
    PointCloudPtr final_corner_map(new pcl::PointCloud<PointType>);
    applyTransform<PointType>(aligned_corner_map, R_final, t_final);
    savePCDFile<PointType>(aligned_map_name + "/CornerMap_aligned.pcd", aligned_corner_map);
    *final_corner_map = *base_corner_map + *aligned_corner_map;
    savePCDFile<PointType>(final_map_name + "/CornerMap.pcd", final_corner_map);

    PointCloudPtr base_surf_map(new pcl::PointCloud<PointType>);
    loadPCDFile<PointType>(base_map_name + "/SurfMap.pcd", base_surf_map);
    PointCloudPtr aligned_surf_map(new pcl::PointCloud<PointType>);
    loadPCDFile<PointType>(aligned_map_name + "/SurfMap.pcd", aligned_surf_map);
    for (size_t i = 0; i < aligned_surf_map->points.size(); ++i) {
        auto& point = aligned_surf_map->points[i];
        int idx = static_cast<int>(point.intensity);
        point.intensity = idx + base_traj_max_idx + 1;
    }
    PointCloudPtr final_surf_map(new pcl::PointCloud<PointType>);
    applyTransform<PointType>(aligned_surf_map, R_final, t_final);
    savePCDFile<PointType>(aligned_map_name + "/SurfMap_aligned.pcd", aligned_surf_map);
    *final_surf_map = *base_surf_map + *aligned_surf_map;
    savePCDFile<PointType>(final_map_name + "/SurfMap.pcd", final_surf_map);

    PointCloudPtr final_global_map(new pcl::PointCloud<PointType>);
    *final_global_map = *final_corner_map + *final_surf_map;
    savePCDFile<PointType>(final_map_name + "/GlobalMap.pcd", final_global_map);
    getBoundary(final_global_map).dumpConfig(final_map_name + "/map.ini");

}