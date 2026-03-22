#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tinyxml2.h>
#include <ros/package.h>
#include <tf2/LinearMath/Quaternion.h>

// 定义 Action Client 类型
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// 从XML文件读取最后一个航点的坐标和朝向
bool getLastWaypointFromXML(const std::string& xml_path, 
                           double& x, double& y, 
                           double& ori_x, double& ori_y, double& ori_z, double& ori_w) {
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(xml_path.c_str()) != tinyxml2::XML_SUCCESS) {
        ROS_ERROR("Failed to load XML file: %s", xml_path.c_str());
        return false;
    }

    // 查找根节点<douglas_downsample>
    tinyxml2::XMLElement* root = doc.FirstChildElement("douglas_downsample");
    if (!root) {
        ROS_ERROR("XML file has no <douglas_downsample> root element");
        return false;
    }

    // 遍历所有<Waypoint>，记录最后一个完整航点（包含位置和朝向）
    tinyxml2::XMLElement* last_valid_waypoint = nullptr;
    tinyxml2::XMLElement* waypoint = root->FirstChildElement("Waypoint");
    while (waypoint) {
        // 检查是否包含完整的位置（Pos_x, Pos_y）和朝向（Ori_x~Ori_w）
        if (waypoint->FirstChildElement("Pos_x") && waypoint->FirstChildElement("Pos_y") &&
            waypoint->FirstChildElement("Ori_x") && waypoint->FirstChildElement("Ori_y") &&
            waypoint->FirstChildElement("Ori_z") && waypoint->FirstChildElement("Ori_w")) {
            last_valid_waypoint = waypoint;
        }
        waypoint = waypoint->NextSiblingElement("Waypoint");
    }

    // 提取最后一个航点的位置和朝向
    if (last_valid_waypoint) {
        // 位置
        last_valid_waypoint->FirstChildElement("Pos_x")->QueryDoubleText(&x);
        last_valid_waypoint->FirstChildElement("Pos_y")->QueryDoubleText(&y);
        // 朝向（四元数）
        last_valid_waypoint->FirstChildElement("Ori_x")->QueryDoubleText(&ori_x);
        last_valid_waypoint->FirstChildElement("Ori_y")->QueryDoubleText(&ori_y);
        last_valid_waypoint->FirstChildElement("Ori_z")->QueryDoubleText(&ori_z);
        last_valid_waypoint->FirstChildElement("Ori_w")->QueryDoubleText(&ori_w);

        ROS_INFO("Last waypoint loaded - Pos: (%.2f, %.2f), Ori: (%.4f, %.4f, %.4f, %.4f)",
                 x, y, ori_x, ori_y, ori_z, ori_w);
        return true;
    } else {
        ROS_ERROR("No valid waypoints with orientation found in XML");
        return false;
    }
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "nav_goal_from_xml");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 创建 Action Client，连接 move_base 服务器
    MoveBaseClient ac("move_base", true);

    // 等待 move_base 服务器启动（超时10秒）
    ROS_INFO("Waiting for move_base action server...");
    if (!ac.waitForServer(ros::Duration(10.0))) {
        ROS_FATAL("Timeout waiting for move_base server. Exiting.");
        return -1;
    }
    ROS_INFO("Connected to move_base server.");

    // 获取XML文件路径（与path_optimizer保持一致）
    std::string xml_path;
    std::string pkg_path = ros::package::getPath("rpp_pkg");
    pnh.param<std::string>("xml_path", xml_path, pkg_path + "/path_points/downsampled_path.xml");

    // 读取最后一个航点的位置和朝向
    double goal_x, goal_y;
    double ori_x, ori_y, ori_z, ori_w;  // 四元数朝向
    if (!getLastWaypointFromXML(xml_path, goal_x, goal_y, ori_x, ori_y, ori_z, ori_w)) {
        ROS_FATAL("Failed to get goal from XML, exiting");
        return -1;
    }

    // 校验位置数据
    if (std::isnan(goal_x) || std::isnan(goal_y)) {
        ROS_FATAL("Invalid goal position from XML (contains NaN), exiting");
        return -1;
    }

    // 校验并修复四元数
    if (std::isnan(ori_x) || std::isnan(ori_y) || std::isnan(ori_z) || std::isnan(ori_w)) {
        ROS_WARN("Invalid quaternion from XML (contains NaN), using default orientation (no rotation)");
        ori_x = 0.0;
        ori_y = 0.0;
        ori_z = 0.0;
        ori_w = 1.0;
    }

    // 归一化四元数（确保长度为1）
    double norm = sqrt(ori_x*ori_x + ori_y*ori_y + ori_z*ori_z + ori_w*ori_w);
    if (fabs(norm - 1.0) > 1e-3) {
        ROS_WARN("Quaternion not normalized (norm=%.4f), normalizing...", norm);
        ori_x /= norm;
        ori_y /= norm;
        ori_z /= norm;
        ori_w /= norm;
    }

    // 构造导航目标
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";  // 目标在地图坐标系下
    goal.target_pose.header.stamp = ros::Time::now();

    // 设置位置
    goal.target_pose.pose.position.x = goal_x;
    goal.target_pose.pose.position.y = goal_y;
    goal.target_pose.pose.position.z = 0.0;

    // 设置朝向
    goal.target_pose.pose.orientation.x = ori_x;
    goal.target_pose.pose.orientation.y = ori_y;
    goal.target_pose.pose.orientation.z = ori_z;
    goal.target_pose.pose.orientation.w = ori_w;

    // 发送目标给 move_base
    ROS_INFO("Sending navigation goal - Pos: (%.2f, %.2f), Ori: (%.4f, %.4f, %.4f, %.4f)",
             goal_x, goal_y, ori_x, ori_y, ori_z, ori_w);
    ac.sendGoal(goal);

    // 等待目标完成（超时60秒，可根据需求调整）
    ROS_INFO("Waiting for goal completion...");
    if (ac.waitForResult(ros::Duration(60.0))) {
        // 处理结果
        actionlib::SimpleClientGoalState state = ac.getState();
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Goal reached successfully!");
        } else {
            ROS_WARN("Goal failed with state: %s", state.toString().c_str());
        }
    } else {
        ROS_WARN("Goal did not complete within 60 seconds.");
        // 可选：取消未完成的目标
        ac.cancelGoal();
        ROS_INFO("Goal canceled.");
    }

    return 0;
}