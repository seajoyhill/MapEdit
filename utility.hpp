#pragma once
#include <string>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
using PointType = pcl::PointXYZI;
using PointCloudPtr = pcl::PointCloud<PointType>::Ptr;

template <typename T>
bool loadPCDFile(const std::string& filename, typename pcl::PointCloud<T>::Ptr& cloud)
{
    if (pcl::io::loadPCDFile<T>(filename, *cloud) == -1) {
        std::cerr << "Couldn't read file " << filename << std::endl;
        return false;
    }
    std::cout << "Loaded " << cloud->points.size() << " data points from " << filename << std::endl;
    return true;
}

template <typename T>
bool savePCDFile(const std::string& filename, typename pcl::PointCloud<T>::Ptr& cloud)
{
    try {
        if (pcl::io::savePCDFileASCII(filename, *cloud) == -1) {
            std::cerr << "Failed to save point cloud to " << filename << std::endl;
            return false;
        }
    } catch (const std::exception& e) {
        std::cerr << filename << " " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "Unknown error occurred while saving point cloud to " << filename << std::endl;
        return false;
    }

    std::cout << "Saved " << cloud->points.size() << " data points to " << filename << std::endl;
    return true;
}

template <typename T>
void convertToROSCoordinate(typename pcl::PointCloud<T>::Ptr& cloud)
{
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        auto& point = cloud->points[i];
        double x = point.x;
        double y = point.y;
        double z = point.z;
        point.y = x;
        point.z = y;
        point.x = z;
    }
}

template <typename T>
void convertToLeGOLOAMCoordinate(typename pcl::PointCloud<T>::Ptr& cloud)
{
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        auto& point = cloud->points[i];
        double x = point.x;
        double y = point.y;
        double z = point.z;
        point.z = x;
        point.x = y;
        point.y = z;
    }
}

template <typename T>
void applyTransform(const typename pcl::PointCloud<T>::Ptr& cloudIn, typename pcl::PointCloud<T>::Ptr& cloudOut,
                    const Eigen::Matrix3f& R, const Eigen::Vector3f& t)
{
    cloudOut->resize(cloudIn->points.size());
    for (size_t i = 0; i < cloudIn->points.size(); ++i) {
        const auto& point = cloudIn->points[i];
        Eigen::Vector3f pt(point.x, point.y, point.z);
        Eigen::Vector3f pt_transformed = R * pt + t;
        auto& new_point = cloudOut->points[i];
        new_point.x = pt_transformed.x();
        new_point.y = pt_transformed.y();
        new_point.z = pt_transformed.z();
        new_point.intensity = point.intensity;
    }
}

template <typename T>
void applyTransform(typename pcl::PointCloud<T>::Ptr& cloud, const Eigen::Matrix3f& R, const Eigen::Vector3f& t)
{
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        auto& point = cloud->points[i];
        Eigen::Vector3f pt(point.x, point.y, point.z);
        Eigen::Vector3f pt_transformed = R * pt + t;
        point.x = pt_transformed.x();
        point.y = pt_transformed.y();
        point.z = pt_transformed.z();
    }
}

/// Load a 4x4 transform matrix from a text file (4 rows × 4 numbers; skips empty lines and `#` comments).
/// Use e.g. `loadTransformFromTxt<float>(path)` or `loadTransformFromTxt<double>(path)`.
template <typename Scalar>
Eigen::Matrix<Scalar, 4, 4> loadTransformFromTxt(const std::string& path)
{
    std::ifstream ifs(path);
    if (!ifs) {
        throw std::runtime_error("loadTransformFromTxt: open failed: " + path);
    }
    Eigen::Matrix<Scalar, 4, 4> T;
    std::string line;
    int row = 0;
    while (std::getline(ifs, line) && row < 4) {
        if (line.empty()) {
            continue;
        }
        const auto p = line.find_first_not_of(" \t\r\n");
        if (p == std::string::npos) {
            continue;
        }
        if (line[p] == '#') {
            continue;
        }
        std::istringstream iss(line);
        for (int col = 0; col < 4; ++col) {
            if (!(iss >> T(row, col))) {
                throw std::runtime_error("loadTransformFromTxt: parse error at row " + std::to_string(row));
            }
        }
        ++row;
    }
    if (row != 4) {
        throw std::runtime_error("loadTransformFromTxt: expected 4 data rows, got " + std::to_string(row));
    }
    return T;
}