// 
// 用matplotlib绘制路径图，用于与全局路径的比对，查看算法效果
//

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "matplotlibcpp.h"
#include <vector>

namespace plt = matplotlibcpp;

// 全局变量：存储全局路径、当前位置、历史轨迹
std::vector<double> global_x, global_y;
double curr_x = 0.0, curr_y = 0.0;
std::vector<double> history_x, history_y;  // 新增：存储历史位置

bool has_global = false;
bool has_curr = false;

// 全局路径回调
void globalCallback(const nav_msgs::Path::ConstPtr& msg) {
    global_x.clear();
    global_y.clear();
    for (const auto& pose : msg->poses) {
        global_x.push_back(pose.pose.position.x);
        global_y.push_back(pose.pose.position.y);
    }
    has_global = !global_x.empty();
}

// 机器人当前位姿回调（同时保存历史）
void robotCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    curr_x = msg->pose.position.x;
    curr_y = msg->pose.position.y;
    has_curr = true;

    // 保存当前位置到历史轨迹（限制历史长度，避免内存占用过大）
    history_x.push_back(curr_x);
    history_y.push_back(curr_y);
    // 只保留最近1000个点（可根据需要调整）
    if (history_x.size() > 1000) {
        history_x.erase(history_x.begin());
        history_y.erase(history_y.begin());
    }
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "path_with_history");
    ros::NodeHandle nh;

    // 订阅话题
    ros::Subscriber sub_global = nh.subscribe("/move_base/GlobalPlanner/plan", 10, globalCallback);
    ros::Subscriber sub_robot = nh.subscribe("/current_pose", 10, robotCallback);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        plt::clf();  // 清空当前图像

        // 1. 绘制全局路径（蓝色）
        if (has_global) {
            plt::plot(global_x, global_y, "b-");
        }

        // 2. 绘制历史轨迹
        if (history_x.size() > 1) {
            // "r:" 表示红色点线，";lw=2" 表示线宽为2（数值越大越粗）
            plt::plot(history_x, history_y, "r:");  
        }

        // 3. 绘制当前位置（红色圆点）
        if (has_curr) {
            std::vector<double> rx, ry;
            rx.push_back(curr_x);
            ry.push_back(curr_y);
            plt::plot(rx, ry, "ro");  // 红色圆点标记当前位置
        }

        // 图像设置
        //plt::title("blue:global_path==red:current_path");
        plt::grid(true);
        plt::pause(0.1);  // 刷新显示

        loop_rate.sleep();
    }

    return 0;
}