#include <pluginlib/class_list_macros.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <fstream>
#include <tinyxml2.h>
#include "rpp_pkg/bspline_curve.h"

namespace rpp_pkg {

class PointGlobalPlanner : public nav_core::BaseGlobalPlanner {
public:
    PointGlobalPlanner();
    PointGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

private:
    std::vector<rpp_pkg::Point2d> readPointsFromXML(const std::string& filename);
    ros::NodeHandle nh_;          // 全局节点句柄
    ros::NodeHandle private_nh_;  // 私有节点句柄（用于读取命名空间内的参数）
    std::string xml_filename_;    // XML文件路径
    rpp_pkg::BSplineCurve bspline_;
    bool initialized_;
    ros::Publisher global_path_pub_; // 全局路径发布器（调试用）
    private:
    //控制发布频率的变量
    void publishPathTimerCallback(const ros::TimerEvent& event);
    ros::Timer publish_timer_;          // 定时器
    double publish_frequency_;          // 发布频率(Hz)
    ros::Time last_publish_time_;       // 上次发布时间
    nav_msgs::Path last_global_path_;   // 缓存的路径消息
};

// 构造函数
PointGlobalPlanner::PointGlobalPlanner() : initialized_(false), private_nh_("~") {
    // 初始化放在initialize函数中
}

// 带参数的构造函数
PointGlobalPlanner::PointGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : private_nh_("~") {
    // 初始化放在initialize函数中
    initialize(name, costmap_ros);
}

// 初始化函数：从参数服务器读取XML路径
void PointGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        // 1. 从私有命名空间读取参数（支持launch文件配置）
        // 格式：<param name="PointGlobalPlanner/xml_path" value="..."/>
        std::string default_path = ros::package::getPath("rpp_pkg") + "/path_points/path_1.xml";
        private_nh_.param<std::string>("xml_path", xml_filename_, default_path);

        // 2. 验证路径是否有效
        tinyxml2::XMLDocument doc;
        if (doc.LoadFile(xml_filename_.c_str()) != tinyxml2::XML_SUCCESS) {
            ROS_WARN("XML file not found at specified path: %s. Using default path: %s",
                     xml_filename_.c_str(), default_path.c_str());
            xml_filename_ = default_path; // 回退到默认路径
        }

        // 读取发布频率参数（默认1Hz）
        private_nh_.param<double>("publish_frequency", publish_frequency_, 10.0);
        if (publish_frequency_ <= 0) {
            ROS_WARN("Invalid publish frequency (%.2f), using default 1Hz", publish_frequency_);
            publish_frequency_ = 1.0;
        }

        // 初始化定时器，按指定频率发布路径
        publish_timer_ = nh_.createTimer(ros::Duration(1.0/publish_frequency_),
                                        &PointGlobalPlanner::publishPathTimerCallback, this);

        // 3. 初始化发布器
        global_path_pub_ = nh_.advertise<nav_msgs::Path>("/move_base/GlobalPlanner/global_plan", 10, true);

        initialized_ = true;
        ROS_INFO("PointGlobalPlanner initialized. XML path: %s", xml_filename_.c_str());
    } else {
        ROS_WARN("PointGlobalPlanner has already been initialized");
    }


}

// 生成路径
bool PointGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR("Planner not initialized! Call initialize() first.");
        return false;
    }

    // 读取XML路径点
    std::vector<rpp_pkg::Point2d> waypoints = readPointsFromXML(xml_filename_);
    if (waypoints.empty()) {
        ROS_FATAL("No valid waypoints loaded from XML!");
        return false;
    }

    // 生成B样条路径
    rpp_pkg::Points3d smoothed_path;
    if (!bspline_.run(waypoints, smoothed_path)) {
        ROS_ERROR("B-spline path generation failed!");
        return false;
    }

    // 填充plan（供move_base局部规划器使用）
    plan.clear();
    nav_msgs::Path global_path_msg;
    global_path_msg.header.stamp = ros::Time::now();
    global_path_msg.header.frame_id = "map";

    for (const auto& pt : smoothed_path) {
        geometry_msgs::PoseStamped pose;
        pose.header = global_path_msg.header;
        pose.pose.position.x = pt.x();
        pose.pose.position.y = pt.y();
        pose.pose.position.z = 0.0;

        tf2::Quaternion quat;
        quat.setRPY(0, 0, pt.theta());
        pose.pose.orientation.x = quat.x();
        pose.pose.orientation.y = quat.y();
        pose.pose.orientation.z = quat.z();
        pose.pose.orientation.w = quat.w();

        plan.push_back(pose);
        global_path_msg.poses.push_back(pose);
    }

    // 新增：仅更新缓存的路径，不直接发布
    last_global_path_ = global_path_msg;
    ROS_INFO("Updated B-spline global path with %zu waypoints", smoothed_path.size());

    return true;
}

// 从XML读取路径点
std::vector<rpp_pkg::Point2d> PointGlobalPlanner::readPointsFromXML(const std::string& filename) {
    std::vector<rpp_pkg::Point2d> points;
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError err = doc.LoadFile(filename.c_str());

    if (err != tinyxml2::XML_SUCCESS) {
        ROS_ERROR("Failed to load XML file: %s (error: %s)", filename.c_str(), doc.ErrorName());
        return points;
    }

    tinyxml2::XMLElement* root = doc.FirstChildElement("douglas_downsample");
    if (!root) {
        ROS_ERROR("XML file has no <douglas_downsample> root element");
        return points;
    }

    tinyxml2::XMLElement* waypoint = root->FirstChildElement("Waypoint");
    while (waypoint) {
        double x, y;
        tinyxml2::XMLElement* pos_x = waypoint->FirstChildElement("Pos_x");
        tinyxml2::XMLElement* pos_y = waypoint->FirstChildElement("Pos_y");
        if (pos_x && pos_y && pos_x->QueryDoubleText(&x) == tinyxml2::XML_SUCCESS &&
            pos_y->QueryDoubleText(&y) == tinyxml2::XML_SUCCESS) {
            // 检查读取的数据是否有效
            if (!std::isnan(x) && !std::isnan(y)) {
                points.emplace_back(x, y);
                ROS_DEBUG("Loaded waypoint: (%.2f, %.2f)", x, y);
            } else {
                ROS_WARN("Skipping invalid waypoint with NaN values");
            }
        } else {
            ROS_WARN("Skipping invalid waypoint (missing Pos_x or Pos_y)");
        }
        waypoint = waypoint->NextSiblingElement("Waypoint");
    }

    ROS_INFO("Successfully loaded %lu points from XML file", points.size());
    return points;
}

// 定时器回调函数，控制路径发布频率
void PointGlobalPlanner::publishPathTimerCallback(const ros::TimerEvent& event) {
    if (!initialized_ || last_global_path_.poses.empty()) {
        return; // 未初始化或无有效路径时不发布
    }
    // 更新时间戳后发布
    last_global_path_.header.stamp = ros::Time::now();
    global_path_pub_.publish(last_global_path_);
    ROS_DEBUG_THROTTLE(1.0, "Publishing global path (periodic) with %zu waypoints", 
                      last_global_path_.poses.size());
}

}  // namespace rpp_pkg


PLUGINLIB_EXPORT_CLASS(rpp_pkg::PointGlobalPlanner, nav_core::BaseGlobalPlanner)