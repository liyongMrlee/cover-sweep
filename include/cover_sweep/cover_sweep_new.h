#ifndef _COVER_SWEEP_
#define _COVER_SWEEP_

#include<iostream>
#include<cmath>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include <Eigen/Eigen>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cover_sweep/jps_search.h> 
#include "tf/transform_listener.h"

#define SWEEP_GAP 0.25
#define MAP_RESELUTION 0.05
#define BORDER_VALUE    80

typedef struct sweep_point
{
    Eigen::Vector2d point;
    bool visited;
}SweepPoint;


class COVER_SWEEP
{
private:
    ros::NodeHandle node_;
    /*ros topic*/
    ros::Subscriber goal_sub_;
    ros::Subscriber map_sub_,border_sub_;
    ros::Publisher expandmap_pub_,expandmap_border_pub_;
    ros::Publisher path_pub_,border_pub_,trajectory_pub_;
    
    friend class JPS_Search;
    std::vector<Eigen::Vector2d>   zone_vec;
    std::vector<std::vector<SweepPoint> >  sweep_point_vec;
    Eigen::Vector2i current_point_index,last_point_index;
    nav_msgs::OccupancyGrid expand_map,expand_map_border;
    nav_msgs::OccupancyGrid static_map,border_map;
    bool plan_finished;
    JPS_Search jps_plan_;
    bool last_path_notLine;
    bool border_status;
    tf::StampedTransform location_transform;
    tf::TransformListener location_listener;
    boost::thread* sweep_thread_;
    float border_min_x = 100,border_min_y = 100,border_max_x = -100,border_max_y = -100;
    bool findLineStatus = false;
    int main_direction = 0;
    bool borderStatus = true;
    bool coverPlanStatus = true;
    bool updateLineStatus = true;
    bool sweepStatus = false;
    bool sweepByCol = false;
    Eigen::Vector2d border_start_point;
    Eigen::Vector2d border_point_last;

public:
    COVER_SWEEP();
    ~COVER_SWEEP();

    /*topic       /move_base_simple/goal     geometry_msgs/PoseStamped类型*/
    /*规划前准备工作*/
    void goalCallBack(const geometry_msgs::PoseStamped::ConstPtr& goal);
    void MapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& map);
    void borderInfoCallBack(const std_msgs::Int32::ConstPtr& border_info);
    void sweepSpin(double transform_publish_period);
    void getBorderPath(nav_msgs::Path& path,std::vector<Eigen::Vector2d> zone);
    bool borderControl(nav_msgs::Path& path,geometry_msgs::Pose robot_pose);
    void markBorderPointIntoMap(nav_msgs::OccupancyGrid& map,geometry_msgs::Pose pose);
    bool isReachTarget(geometry_msgs::Pose pose,Eigen::Vector2d target);
    bool getLocation(geometry_msgs::Pose& pos_now);
    Eigen::Vector2i findInitPoint(geometry_msgs::Pose pose);

    /*纵向清扫算法*/
    bool coverPlanByCol(nav_msgs::Path& path,geometry_msgs::Pose pose);
    bool reFindLineByCol(Eigen::Vector2i& current_index,Eigen::Vector2i& last_index);
    bool find_lineByCol();
    void deleteOutCurvePointByCol(std::vector<SweepPoint>& sweep_line_point);
    bool sweepPlan(nav_msgs::Path& path,geometry_msgs::Pose pose,Eigen::Vector2i& current_index,Eigen::Vector2i& last_index);
    bool find_next_point(Eigen::Vector2i& current_index,nav_msgs::Path& path,Eigen::Vector2i& last_index);
    bool findCurrentLineWithStraight(const Eigen::Vector2i current_index,Eigen::Vector2i& next_index);
    bool findCurrentLineWithStraight(const Eigen::Vector2i current_index,Eigen::Vector2i& next_index,int direction);
    bool findNearLineWithStraight(const Eigen::Vector2i current_index,Eigen::Vector2i& next_index);
    bool findNearLineWithStraight(const Eigen::Vector2i current_index,Eigen::Vector2i& next_index,int direction,float& min_distance);
  
    bool findCurrentLineWithJPS(const Eigen::Vector2i current_index,Eigen::Vector2i& next_index,nav_msgs::Path& path);
    bool findCurrentLineWithJPS(const Eigen::Vector2i current_index,
                                Eigen::Vector2i& next_index,nav_msgs::Path& path,int direction,float& min_distance);
    bool findNearLineWithJPS(const Eigen::Vector2i current_index,Eigen::Vector2i& next_index,nav_msgs::Path& path);
    bool findNearLineWithJPS(const Eigen::Vector2i current_index,
                            Eigen::Vector2i& next_index,nav_msgs::Path& path,int direction,float& min_distance);
    
    /*横向清扫算法*/
    bool coverPlanByRow(nav_msgs::Path& path,geometry_msgs::Pose pose);
    bool reFindLineByRow(Eigen::Vector2i& current_index,Eigen::Vector2i& last_index);
    bool find_lineByRow();
    void deleteOutCurvePointByRow(std::vector<SweepPoint>& sweep_line_point);

    /*共用工具类函数*/
    float direction(Eigen::Vector2d pi,Eigen::Vector2d pj,Eigen::Vector2d pk) //计算向量pkpi和向量pjpi的叉积   
    {  
        return (pi(0)-pk(0))*(pi(1)-pj(1))-(pi(1)-pk(1))*(pi(0)-pj(0));  
    }
    bool isOutCruvePointByCol(const Eigen::Vector2i point_grid,const int direction);
    bool isOutCruvePointByRow(const Eigen::Vector2i point_grid,const int direction);
    float getJpsDistance(Eigen::Vector2d point1,Eigen::Vector2d point2);
    float getEuclideanDistance(Eigen::Vector2d point1,Eigen::Vector2d point2);
    bool isLineEmpty(std::vector<SweepPoint>& line_vec);
    bool collisionCheck(Eigen::Vector2d start,Eigen::Vector2d end,nav_msgs::OccupancyGrid& map,int occupied_value = 100);
    bool isVaildLine(Eigen::Vector2i current_index,Eigen::Vector2i& next_index);
    bool isLineCross(Eigen::Vector2d line1_start,Eigen::Vector2d line1_end,Eigen::Vector2d line2_start,Eigen::Vector2d line2_end);

};

#endif