#ifndef _COVER_SWEEP_
#define _COVER_SWEEP_

#include<iostream>
#include<cmath>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Eigen>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cover_sweep/jps_search.h> 

#define SWEEP_GAP 0.2
#define MAP_RESELUTION 0.05

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
    ros::Subscriber map_sub_;
    ros::Publisher expandmap_pub_;
    ros::Publisher path_pub_;
    
    friend class JPS_Search;
    std::vector<Eigen::Vector2d>   zone_vec;
    std::vector<std::vector<SweepPoint> >  sweep_point_vec;
    Eigen::Vector2i current_point_index,last_point_index;
    nav_msgs::OccupancyGrid expand_map;
    bool plan_finished;
    JPS_Search jps_plan_;
    bool last_path_notLine;

public:
    COVER_SWEEP();
    ~COVER_SWEEP();

    /*topic       /move_base_simple/goal     geometry_msgs/PoseStamped类型*/
    void goalCallBack(const geometry_msgs::PoseStamped::ConstPtr& goal);
    void MapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& map);
    bool coverPlan(std::vector<Eigen::Vector2d>& zone,nav_msgs::Path& path);
    bool full_cover(std::vector<Eigen::Vector2d>& zone_vertx,nav_msgs::Path& path);
    void find_line_point(std::vector<Eigen::Vector2d>& zone_vertx,std::vector<SweepPoint>& sweep_line_point,int index);
    void sweep_path_plan(nav_msgs::Path& path);
    float getManhattanDistance(Eigen::Vector2d point1,Eigen::Vector2d point2);
    float getChebyshevDistance(Eigen::Vector2d point1,Eigen::Vector2d point2);
    float getJpsDistance(Eigen::Vector2d point1,Eigen::Vector2d point2);
    float getEuclideanDistance(Eigen::Vector2d point1,Eigen::Vector2d point2);
    bool isLineEmpty(std::vector<SweepPoint>& line_vec);
    bool isNearLineEmpty(Eigen::Vector2i& current_index);
    void find_next_point(Eigen::Vector2i& current_index,std::vector<Eigen::Vector2d>& path_sweep);
    bool jump(Eigen::Vector2i& current_index,Eigen::Vector2i& next_index,std::vector<Eigen::Vector2d>& next_point_vec,int direction/*1:向上搜索   -1：向下搜索*/,float& max_distance);
    bool find_next_line_start(Eigen::Vector2i& current_index,std::vector<Eigen::Vector2d>& next_point_vec,float& distance);
    bool jump2NextLine(Eigen::Vector2i& current_index,std::vector<Eigen::Vector2d>& next_point_vec,int direction/*1:向右搜索  2：向左搜索*/,float& distance);
    bool getLeftLinePoint_ByStraight(Eigen::Vector2i& current_index,Eigen::Vector2i& next_index,std::vector<Eigen::Vector2d>& next_point_vec);
    bool getRightLinePoint_ByStraight(Eigen::Vector2i& current_index,Eigen::Vector2i& next_index,std::vector<Eigen::Vector2d>& next_point_vec);
    bool collisionCheck(Eigen::Vector2d start,Eigen::Vector2d end);
    bool getManhattanPath(Eigen::Vector2d start,Eigen::Vector2d end,std::vector<Eigen::Vector2d>& manhattan_path);
    bool get3LineRectangle(Eigen::Vector2d start,Eigen::Vector2d end,std::vector<Eigen::Vector2d>& line3rectangle_path,float& distance);
    void getJpsPath(nav_msgs::Path temp_path,std::vector<Eigen::Vector2d>& path_vec);
    bool compy(SweepPoint point1,SweepPoint point2)
    {
        return point1.point(1) > point2.point(1);
    }
};

#endif