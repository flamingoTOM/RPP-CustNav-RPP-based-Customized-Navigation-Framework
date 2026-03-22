// rpp_pkg/include/rpp_pkg/point_global_planner.h
#ifndef RPP_PKG_POINT_GLOBAL_PLANNER_H_
#define RPP_PKG_POINT_GLOBAL_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <fstream>
#include <tinyxml2.h>
#include "rpp_pkg/bspline_curve.h"

namespace rpp_pkg 
{
class PointGlobalPlanner : public nav_core::BaseGlobalPlanner
{
public:
    PointGlobalPlanner();
    PointGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

private:
    std::vector<rpp_pkg::Point2d> readPointsFromXML(const std::string& filename);
    ros::NodeHandle nh_;
    std::string xml_filename_;
    rpp_pkg::BSplineCurve bspline_;
    bool initialized_; 

    void publishPathTimerCallback(const ros::TimerEvent& event);
    ros::Timer publish_timer_;
    double publish_frequency_;
    ros::Time last_publish_time_;
    nav_msgs::Path last_global_path_;

};

 
} 

#endif