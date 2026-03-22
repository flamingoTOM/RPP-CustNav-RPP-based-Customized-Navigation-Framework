/**
 * *********************************************************
 *
 * @file: douglas.cpp
 * @brief: 道格拉斯降采样算法
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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

double epsilon = 0.05; // 降采样阈值

// 计算点到线段的垂直距离
double perpendicularDistance(const geometry_msgs::PoseStamped& point, 
                             const geometry_msgs::PoseStamped& start, 
                             const geometry_msgs::PoseStamped& end) {
    double num = std::abs((end.pose.position.y - start.pose.position.y) * point.pose.position.x - 
                         (end.pose.position.x - start.pose.position.x) * point.pose.position.y + 
                         end.pose.position.x * start.pose.position.y - 
                         end.pose.position.y * start.pose.position.x);
    double den = std::sqrt(std::pow(end.pose.position.y - start.pose.position.y, 2) + 
                         std::pow(end.pose.position.x - start.pose.position.x, 2));
    // 避免除零错误
    if (den < 1e-10) {
        return 0.0;
    }
    return num / den;
}

// 道格拉斯 - 普克算法
void douglasPeucker(const std::vector<geometry_msgs::PoseStamped>& points, 
                    double epsilon, 
                    std::vector<geometry_msgs::PoseStamped>& result) {
    if (points.size() < 2) {
        result = points;
        return;
    }
    
    double dmax = 0;
    int index = 0;
    int end = points.size() - 1;

    // 找到距离首尾连线最远的点
    for (int i = 1; i < end; ++i) {
        double d = perpendicularDistance(points[i], points[0], points[end]);
        if (d > dmax) {
            index = i;
            dmax = d;
        }
    }

    if (dmax > epsilon) {
        std::vector<geometry_msgs::PoseStamped> results1, results2;
        // 递归处理前半部分点
        douglasPeucker(std::vector<geometry_msgs::PoseStamped>(points.begin(), points.begin() + index + 1), epsilon, results1);
        // 递归处理后半部分点
        douglasPeucker(std::vector<geometry_msgs::PoseStamped>(points.begin() + index, points.end()), epsilon, results2);
        
        // 合并结果，避免重复添加第一个点
        result.insert(result.end(), results1.begin(), results1.end() - 1);
        result.insert(result.end(), results2.begin(), results2.end());
    } else {
        // 如果最大距离小于等于阈值，保留首尾两点
        result.push_back(points[0]);
        result.push_back(points[end]);
    }
}

// 保存采样点到 XML
void saveDownsampledPath(const std::vector<geometry_msgs::PoseStamped>& downsampled_path, const std::string& filename) {
    std::ofstream file(filename);
    if (file.is_open()) {
        // 写入XML头部
        file << "<douglas_downsample>" << std::endl;
        
        // 遍历每个路径点
        for (size_t i = 0; i < downsampled_path.size(); ++i) {
            const auto& pose = downsampled_path[i];
            
            file << "  <Waypoint>" << std::endl;
            file << "    <Name>" << (i + 1) << "</Name>" << std::endl;
            
            file << "    <Pos_x>" << pose.pose.position.x << "</Pos_x>" << std::endl;
            file << "    <Pos_y>" << pose.pose.position.y << "</Pos_y>" << std::endl;
            file << "    <Pos_z>" << pose.pose.position.z << "</Pos_z>" << std::endl;
            file << "    <Ori_x>" << pose.pose.orientation.x << "</Ori_x>" << std::endl;
            file << "    <Ori_y>" << pose.pose.orientation.y << "</Ori_y>" << std::endl;
            file << "    <Ori_z>" << pose.pose.orientation.z << "</Ori_z>" << std::endl;
            file << "    <Ori_w>" << pose.pose.orientation.w << "</Ori_w>" << std::endl;

            file << "  </Waypoint>" << std::endl;
        }
        // 写入XML尾部
        file << "</douglas_downsample>" << std::endl;
        file.close();
        ROS_INFO("Downsampled path saved to %s", filename.c_str());
    } else {
        ROS_ERROR("Unable to open file %s", filename.c_str());
    }
}

// 回调函数，处理全局路径消息
void globalPathCallback(const nav_msgs::Path::ConstPtr& msg, ros::Publisher& downsampled_points_pub) {
    if (msg->poses.empty()) {
        ROS_WARN("Received empty path, skipping downsampling");
        return;
    }
    
    std::vector<geometry_msgs::PoseStamped> global_path = msg->poses;
    std::vector<geometry_msgs::PoseStamped> downsampled_path;

    douglasPeucker(global_path, epsilon, downsampled_path);
    
    // 检查降采样后的路径是否有效
    if (downsampled_path.size() < 2) {
        ROS_WARN("Downsampled path has less than 2 points, using original path instead");
        downsampled_path = global_path;
    }

    // 获取功能包的绝对路径
    std::string pkg_path = ros::package::getPath("rpp_pkg");
    // 确保目录存在
    system(("mkdir -p " + pkg_path + "/path_points").c_str());
    // 拼接保存文件的完整路径
    std::string file_path = pkg_path + "/path_points/downsampled_path.xml";

    // 保存采样点到文件
    saveDownsampledPath(downsampled_path, file_path);

    // 创建降采样后的路径消息
    nav_msgs::Path downsampled_points_msg;
    downsampled_points_msg.header = msg->header;
    downsampled_points_msg.poses = downsampled_path;

    // 发布降采样后的路径点
    downsampled_points_pub.publish(downsampled_points_msg);
    
    ROS_INFO("Path downsampled: %lu points -> %lu points", global_path.size(), downsampled_path.size());
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "douglas_node");
    ros::NodeHandle nh;
    
    // 从参数服务器获取降采样阈值
    nh.param("douglas_epsilon", epsilon, 0.1);
    
    ros::Publisher downsampled_points_pub = nh.advertise<nav_msgs::Path>("downsampled_points", 10);
    ros::Subscriber global_path_sub = nh.subscribe<nav_msgs::Path>(
        "/move_base/GlobalPlanner/plan", 
        10, 
        boost::bind(globalPathCallback, _1, boost::ref(downsampled_points_pub))
    );

    ROS_INFO("Douglas-Peucker path downsampler started with epsilon: %f", epsilon);
    ros::spin();
    return 0;
}