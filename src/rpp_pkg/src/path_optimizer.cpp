/**
 * *********************************************************
 *
 * @file: path_optimizer.cpp
 * @brief: 从XML文件读取路径点并生成B样条平滑曲线（验证姿态影响）
 * @author: flamingo
 * @date: 2025-7-29
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <cmath>
#include <fstream>
#include <ros/package.h>
#include <tinyxml2.h>  // 用于解析XML文件
#include "rpp_pkg/bspline_curve.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>

// 检查值是否为NaN
inline bool isNaN(double value) {
    return std::isnan(value);
}

// 从XML文件读取路径点
std::vector<rpp_pkg::Point2d> readPointsFromXML(const std::string& filename) {
    std::vector<rpp_pkg::Point2d> points;
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError err = doc.LoadFile(filename.c_str());

    if (err != tinyxml2::XML_SUCCESS) {
        ROS_ERROR("Failed to load XML file: %s (error: %s)", filename.c_str(), doc.ErrorName());
        return points;
    }

    // 查找根节点<douglas_downsample>
    tinyxml2::XMLElement* root = doc.FirstChildElement("douglas_downsample");
    if (!root) {
        ROS_ERROR("XML file has no <douglas_downsample> root element");
        return points;
    }

    // 遍历所有<Waypoint>子节点
    tinyxml2::XMLElement* waypoint = root->FirstChildElement("Waypoint");
    while (waypoint) {
        // 读取位置信息（x, y）
        double x, y;
        tinyxml2::XMLElement* pos_x = waypoint->FirstChildElement("Pos_x");
        tinyxml2::XMLElement* pos_y = waypoint->FirstChildElement("Pos_y");
        if (pos_x && pos_y && pos_x->QueryDoubleText(&x) == tinyxml2::XML_SUCCESS &&
            pos_y->QueryDoubleText(&y) == tinyxml2::XML_SUCCESS) {
            
            // 检查是否为有效值
            if (isNaN(x) || isNaN(y)) {
                ROS_WARN("Skipping waypoint with NaN values");
                waypoint = waypoint->NextSiblingElement("Waypoint");
                continue;
            }
            
            points.emplace_back(x, y);  
            ROS_DEBUG("Loaded waypoint: (%.2f, %.2f)", x, y);
        } else {
            ROS_WARN("Skipping invalid waypoint (missing Pos_x or Pos_y)");
        }
        waypoint = waypoint->NextSiblingElement("Waypoint");
    }

    ROS_INFO("Successfully loaded %lu points from XML file", points.size());
    return points;
}

// 验证B样条路径点位置是否有效（暂时不验证theta，因为我们将使用固定值）
bool validateBSplinePositions(const rpp_pkg::Points3d& path) {
    for (size_t i = 0; i < path.size(); ++i) {
        const auto& pt = path[i];
        if (isNaN(pt.x()) || isNaN(pt.y())) {
            ROS_ERROR("Invalid B-spline position at index %zu: (%.2f, %.2f)", 
                      i, pt.x(), pt.y());
            return false;
        }
    }
    return true;
}

// 生成B样条曲线并发布（使用固定姿态）
bool generateBSplinePath(const std::vector<rpp_pkg::Point2d>& points, 
                         nav_msgs::Path& bspline_msg,
                         rpp_pkg::BSplineCurve& bspline) {
    if (points.size() < 2) {  // B样条至少需要2个点
        ROS_ERROR("Not enough points to generate B-spline (need at least 2), current: %lu", points.size());
        return false;
    }

    // 生成平滑路径（只关注位置，忽略角度）
    rpp_pkg::Points3d smoothed_path;  // 包含x, y, theta（角度）
    if (!bspline.run(points, smoothed_path)) {
        ROS_ERROR("B-spline generation failed");
        return false;
    }

    // 验证路径点位置有效性
    if (!validateBSplinePositions(smoothed_path)) {
        ROS_ERROR("Generated B-spline path contains invalid positions");
        return false;
    }

    // 转换为nav_msgs::Path消息
    bspline_msg.header.stamp = ros::Time::now();
    bspline_msg.header.frame_id = "map";  // 坐标系与输入路径一致
    bspline_msg.poses.clear();

    // 计算路径总方向作为固定角度（从第一个点到最后一个点）
    double fixed_yaw = 0.0;
    if (smoothed_path.size() >= 2) {
        const auto& first = smoothed_path.front();
        const auto& last = smoothed_path.back();
        double dx = last.x() - first.x();
        double dy = last.y() - first.y();
        fixed_yaw = atan2(dy, dx);  // 计算整体方向角
        
        // 确保角度在[-π, π]范围内
        if (fixed_yaw > M_PI) fixed_yaw -= 2 * M_PI;
        if (fixed_yaw < -M_PI) fixed_yaw += 2 * M_PI;
        
        ROS_INFO("Using fixed yaw angle: %.2f radians (%.2f degrees)", 
                 fixed_yaw, fixed_yaw * 180.0 / M_PI);
    }

    for (const auto& pt : smoothed_path) {
        geometry_msgs::PoseStamped pose;
        pose.header = bspline_msg.header;
        pose.pose.position.x = pt.x();
        pose.pose.position.y = pt.y();
        pose.pose.position.z = 0.0;  // 2D路径，z=0

        // 使用固定角度生成四元数，替代pt.theta()
        tf2::Quaternion quat;
        quat.setRPY(0, 0, fixed_yaw);  // 固定偏航角，避免使用可能含NaN的theta
        pose.pose.orientation = tf2::toMsg(quat);

        bspline_msg.poses.push_back(pose);
    }

    ROS_INFO("B-spline path generated with %lu points (using fixed yaw angle)", smoothed_path.size());
    return true;
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "path_optimizer_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); 

    // 从参数服务器获取XML文件路径
    std::string xml_filename;
    std::string pkg_path = ros::package::getPath("rpp_pkg");
    pnh.param<std::string>("xml_path", xml_filename, pkg_path + "/path_points/downsampled_path.xml");
    
    // 发布频率参数
    double publish_rate;
    pnh.param<double>("publish_rate", publish_rate, 10.0);
    if (publish_rate <= 0) {
        ROS_WARN("Invalid publish rate (%.2f), using default 10Hz", publish_rate);
        publish_rate = 10.0;
    }

    // 读取XML文件中的路径点
    std::vector<rpp_pkg::Point2d> waypoints = readPointsFromXML(xml_filename);
    if (waypoints.empty()) {
        ROS_FATAL("No valid waypoints loaded, exiting");
        return -1;
    }

    // 创建B样条路径发布者
    ros::Publisher bspline_pub = nh.advertise<nav_msgs::Path>("/move_base/GlobalPlanner/plan", 10, true);
    
    // 初始化B样条对象
    rpp_pkg::BSplineCurve bspline;

    // 生成并发布B样条路径
    nav_msgs::Path bspline_msg;
    if (!generateBSplinePath(waypoints, bspline_msg, bspline)) {
        ROS_FATAL("Failed to generate initial B-spline path, exiting");
        return -1;
    }
    bspline_pub.publish(bspline_msg);

    // 循环发布，确保订阅者能接收到
    ros::Rate rate(publish_rate);  // 指定频率发布
    while (ros::ok()) {
        // 定期更新时间戳并发布，保持路径活跃
        bspline_msg.header.stamp = ros::Time::now();
        bspline_pub.publish(bspline_msg);
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
