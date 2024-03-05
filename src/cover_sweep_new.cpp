#include "cover_sweep/cover_sweep_new.h"

COVER_SWEEP::COVER_SWEEP():
            plan_finished(false),
            last_path_notLine(false),
            border_status(true)
{
    goal_sub_ = node_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, boost::bind(&COVER_SWEEP::goalCallBack, this, _1));
    map_sub_ = node_.subscribe<nav_msgs::OccupancyGrid>("/map", 1, boost::bind(&COVER_SWEEP::MapCallBack, this, _1));
    path_pub_ = node_.advertise<nav_msgs::Path>("/sweep_path", 1, true);
    border_pub_ = node_.advertise<nav_msgs::OccupancyGrid>("/border_map",1,true);
    expandmap_pub_ = node_.advertise<nav_msgs::OccupancyGrid>("/expandmap",1,true);
    expandmap_border_pub_ = node_.advertise<nav_msgs::OccupancyGrid>("/expandmap_border",1,true);
    trajectory_pub_ = node_.advertise<nav_msgs::Path>("/trajectory", 1, true);
    border_sub_ = node_.subscribe<std_msgs::Int32>("/border_info", 1, boost::bind(&COVER_SWEEP::borderInfoCallBack, this, _1));
    JPS_Search* jps_plan_ = new JPS_Search;
    sweep_thread_ = new boost::thread(boost::bind(&COVER_SWEEP::sweepSpin, this, 0.04));
}

COVER_SWEEP::~COVER_SWEEP()
{
    
}

void COVER_SWEEP::sweepSpin(double transform_publish_period)
{
    ros::Rate r(25.0);
    nav_msgs::Path path,robot_trajectory;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";
    geometry_msgs::Pose robot_pose;
    static bool borderStatus_last = true;
    bool zone_init = false;
    while(ros::ok())
    {
        if(zone_vec.size() != 2)  {r.sleep();continue;}
        if(getLocation(robot_pose))
        {
            geometry_msgs::PoseStamped temp_pose;
            temp_pose.pose = robot_pose;
            robot_trajectory.poses.push_back(temp_pose);
            robot_trajectory.header.stamp = ros::Time::now();
            robot_trajectory.header.frame_id = "map";
            trajectory_pub_.publish(robot_trajectory);
            //先获取分区，并沿边
            // if(path.poses.empty())     getBorderPath(path,zone_vec);
            if(!zone_init)
            {
                getBorderPath(path,zone_vec);
                zone_init = true;
            }
            if(!borderStatus)
            {
                if(borderStatus_last && !borderStatus)  
                {
                    border_start_point << robot_pose.position.x,robot_pose.position.y;
                    // border_min_x = 100,border_min_y = 100,border_max_x = -100,border_max_y = -100;
                    ROS_INFO("border start point x = %f,y = %f",robot_pose.position.x,robot_pose.position.y);
                }
                borderStatus = borderControl(path,robot_pose);
                if(!borderStatus)               coverPlanStatus = false;
                if(!sweep_point_vec.empty())    updateLineStatus = false;
                if(border_max_x - border_min_x >= border_max_y - border_min_y)        sweepByCol = false;
                else                                                                  sweepByCol = true;      
                borderStatus_last = borderStatus;
            }
            else
            {
                //再动态获取弓扫路径
                if(!coverPlanStatus)
                {
                    if(sweepByCol)      coverPlanStatus = coverPlanByCol(path,robot_pose);
                    else                coverPlanStatus = coverPlanByRow(path,robot_pose);
                    if(coverPlanStatus)
                    {
                        zone_vec.clear();
                        sweep_point_vec.clear();
                        zone_init = false;borderStatus = true;
                        findLineStatus = false;updateLineStatus = true;
                        border_min_x = 100,border_min_y = 100,border_max_x = -100,border_max_y = -100;
                        border_map = static_map;
                        border_point_last << -100,-100;
                        main_direction = 0;
                    }
                }
            }
        }
        r.sleep();
    }
}

void COVER_SWEEP::getBorderPath(nav_msgs::Path& path,std::vector<Eigen::Vector2d> zone)
{
    float min_x = std::min(zone[0](0),zone[1](0));
    float min_y = std::min(zone[0](1),zone[1](1));
    float max_x = std::max(zone[0](0),zone[1](0));
    float max_y = std::max(zone[0](1),zone[1](1));
    std::vector<Eigen::Vector2d>  zone_vertx;
    zone_vertx.push_back(Eigen::Vector2d(min_x,min_y));
    zone_vertx.push_back(Eigen::Vector2d(max_x,min_y));
    zone_vertx.push_back(Eigen::Vector2d(max_x,max_y));
    zone_vertx.push_back(Eigen::Vector2d(min_x,max_y));
    zone_vertx.push_back(Eigen::Vector2d(min_x,min_y));
    for(int i = 0;i< zone_vertx.size();i++)
    {
        geometry_msgs::PoseStamped temp;
        temp.pose.position.x = zone_vertx[i](0);
        temp.pose.position.y = zone_vertx[i](1);
        path.poses.push_back(temp);
    }
    path_pub_.publish(path);
}

//沿边结束后接收沿边地图
void COVER_SWEEP::MapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& _map)
{
    ROS_INFO("I have recived static map");
    static_map = *_map;
    jps_plan_.expandMap(static_map,expand_map);
    expandmap_pub_.publish(expand_map);
    jps_plan_.mapDataInit(expand_map);
    border_map = static_map;
}

void COVER_SWEEP::borderInfoCallBack(const std_msgs::Int32::ConstPtr& border_info)
{
    if(border_info->data == 1)       //进入沿边状态
    {
        borderStatus = false;
        border_point_last << -100,-100;
        ROS_INFO("border status!");
    }
}

void COVER_SWEEP::goalCallBack(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
    ROS_INFO("I recived new goal!");
    Eigen::Vector2d point(goal->pose.position.x,goal->pose.position.y);
    zone_vec.push_back(point);
}

bool COVER_SWEEP::borderControl(nav_msgs::Path& path,geometry_msgs::Pose robot_pose)
{
    static bool /*start_point_reach = false,*/start_point_leave = false,start_point_loop = false;
    // Eigen::Vector2d border_start_point(path.poses.front().pose.position.x,path.poses.front().pose.position.y);
    // Eigen::Vector2d border_start_point(robot_pose.position.x,robot_pose.position.y);
    // static geometry_msgs::Pose robot_pose_last;

    if(robot_pose.position.x < border_min_x) border_min_x = robot_pose.position.x;
    if(robot_pose.position.y < border_min_y) border_min_y = robot_pose.position.y;
    if(robot_pose.position.x > border_max_x) border_max_x = robot_pose.position.x;
    if(robot_pose.position.y > border_max_y) border_max_y = robot_pose.position.y;        

    // if(sqrt(pow(robot_pose.position.x - robot_pose_last.position.x,2) + 
    //         pow(robot_pose.position.y - robot_pose_last.position.y,2)) > 0.04)
    // {
        //大于上次保存位姿距离0.05米，则标记到地图中，并更新robot_pose_last
        markBorderPointIntoMap(border_map,robot_pose);
        // robot_pose_last = robot_pose;
    // }
    if(!start_point_leave && !isReachTarget(robot_pose,border_start_point))
    {
        start_point_leave = true;
        ROS_INFO("robot leave start point");
    }
    // else if(start_point_leave && !isReachTarget(robot_pose,border_start_point))
    // {
    //     markBorderPointIntoMap(border_map,robot_pose);
    // }
    else if(start_point_leave && isReachTarget(robot_pose,border_start_point))
    {
        start_point_loop = true;
        border_point_last = border_start_point;
        markBorderPointIntoMap(border_map,robot_pose);
        ROS_INFO("robot border loop ok,min_x = %f,min_y = %f,max_x = %f,max_y = %f",
                    border_min_x,border_min_y,border_max_x,border_max_y);
        jps_plan_.expandMap(border_map,expand_map_border,BORDER_VALUE);
        expandmap_border_pub_.publish(expand_map_border);
        border_pub_.publish(border_map);
        // start_point_reach = false,
        path.poses.clear();
        // path.poses.erase(path.poses.begin(),path.poses.begin() + path.poses.size() -1);
        start_point_leave = false,
        start_point_loop = false;
        return true;
    }
    return false;
}

void COVER_SWEEP::markBorderPointIntoMap(nav_msgs::OccupancyGrid& map,geometry_msgs::Pose pose)
{
    /*Eigen::Vector2d border_point;
    static Eigen::Vector2d border_point_last(-100,-100);
    static float yaw_last = -100;
    float robot_radius = 0.2;
    float yaw = tf::getYaw(pose.orientation);
    border_point << pose.position.x + robot_radius*sin(yaw),
                    pose.position.y - robot_radius*cos(yaw);
    if(sqrt(pow(border_point(0) - border_point_last(0),2) + pow(border_point(1) - border_point_last(1),2)) > 0.04 ||
        fabs(yaw - yaw_last) * 180 /M_PI > 10)
    {
        // Eigen::Vector2i robot_pose_grid(jps_plan_.world2grid(Eigen::Vector2d(pose.position.x,pose.position.y)));
        Eigen::Vector2i robot_pose_grid(jps_plan_.world2grid(border_point));
        if(jps_plan_.isVaildCell(robot_pose_grid))
        {
            int point_index = jps_plan_.coord2Id(robot_pose_grid);
            map.data[point_index] = BORDER_VALUE;
        }
        border_point_last = border_point;
        yaw_last = yaw;
    }*/

    Eigen::Vector2d border_point;
    // static Eigen::Vector2d border_point_last(-100,-100);
    border_point << pose.position.x,
                    pose.position.y;
    if(border_point_last(0) == -100 && border_point_last(1) == -100)
    {
        border_point_last = border_point;
        return;
    }
    float point_distance = sqrt(pow(border_point(0) - border_point_last(0),2) + pow(border_point(1) - border_point_last(1),2));
    if(point_distance > 0.04 )
    {
        int point_num = (int)(point_distance / 0.04) + 1;
        for(int i = 0;i < point_num;i++)
        {
            Eigen::Vector2d temp_point;
            temp_point << border_point(0) + i*(border_point_last(0) - border_point(0)) / point_num,
                          border_point(1) + i*(border_point_last(1) - border_point(1)) / point_num;
            Eigen::Vector2i robot_pose_grid(jps_plan_.world2grid(temp_point));
            if(jps_plan_.isVaildCell(robot_pose_grid))
            {
                int point_index = jps_plan_.coord2Id(robot_pose_grid);
                map.data[point_index] = BORDER_VALUE;
            }
        }
        border_point_last = border_point;
    }

}

bool COVER_SWEEP::isReachTarget(geometry_msgs::Pose pose,Eigen::Vector2d target)
{
    return ((fabs(pose.position.x - target(0)) < 0.1) && (fabs(pose.position.y - target(1)) < 0.1));
}

bool COVER_SWEEP::getLocation(geometry_msgs::Pose& pos_now)
{
  try
  {
    location_listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(2.0));
    location_listener.lookupTransform("/map", "/base_link", ros::Time(0), location_transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_INFO("robot_server--->can not get base_link to map!");
    return false;
  }
    pos_now.position.x = static_cast<double>(location_transform.getOrigin().x());
    pos_now.position.y = static_cast<double>(location_transform.getOrigin().y());
    pos_now.orientation.x = static_cast<double>(location_transform.getRotation().x());
    pos_now.orientation.y = static_cast<double>(location_transform.getRotation().y());
    pos_now.orientation.z = static_cast<double>(location_transform.getRotation().z());
    pos_now.orientation.w = static_cast<double>(location_transform.getRotation().w());
    return true;
}

Eigen::Vector2i COVER_SWEEP::findInitPoint(geometry_msgs::Pose pose)
{
    float min_distance = 100;
    Eigen::Vector2i min_distance_point;
    for(int i = 0;i<sweep_point_vec.size();i++)
    {
        for(int j =0;j < sweep_point_vec[i].size();j++)
        {
            if(sweep_point_vec[i][j].visited)   continue;
            Eigen::Vector2d robot_pose(pose.position.x,pose.position.y);
            Eigen::Vector2d temp_point(sweep_point_vec[i][j].point);
            float distance = getEuclideanDistance(robot_pose,temp_point);
            if(distance < min_distance)
            {
                min_distance = distance;
                min_distance_point << i,j;
            }
        }
    }
    return min_distance_point;
}


/*纵向清扫算法：搜索配对点、更新配对点、规划路径*/
bool COVER_SWEEP::coverPlanByCol(nav_msgs::Path& path,geometry_msgs::Pose pose)
{
    static Eigen::Vector2i current_index(-1,-1);
    static Eigen::Vector2i last_index(-1,-1);
    if(!findLineStatus)
    {  
        findLineStatus = find_lineByCol();
        ROS_INFO("Sweep point vec after init find line:");
        for(int i = 0;i < sweep_point_vec.size();i++)
        {
            for(int j = 0;j < sweep_point_vec[i].size();j++)
            {
                ROS_INFO("line = %d,index = %d,x = %f,y = %f,status = %d",i,j+1,sweep_point_vec[i][j].point(0),sweep_point_vec[i][j].point(1),sweep_point_vec[i][j].visited);
            }
            ROS_INFO("-----------------------------------");
        }
        /*int direction = 0;
        for(int i = 0;i < sweep_point_vec.size();i++)
        {
            // ROS_INFO("i = %d,size = %d",i,sweep_point_vec[i].size());
            if(i %2 == 0)
            {
                for(int j = 0;j < sweep_point_vec[i].size();j++)
                {
                    geometry_msgs::PoseStamped temp;
                    temp.pose.position.x = sweep_point_vec[i][j].point(0);
                    temp.pose.position.y = sweep_point_vec[i][j].point(1);
                    path.poses.push_back(temp);
                }
            }
            else
            {
                for(int j = sweep_point_vec[i].size() - 1;j >=0;j--)
                {
                    geometry_msgs::PoseStamped temp;
                    temp.pose.position.x = sweep_point_vec[i][j].point(0);
                    temp.pose.position.y = sweep_point_vec[i][j].point(1);
                    path.poses.push_back(temp);
                }
            }

        }*/
    }
    else if(!updateLineStatus)         //更新sweep_point_vec,并更新current_index 
    {
        updateLineStatus = reFindLineByCol(current_index,last_index);
        if(current_index(0) != -1 && current_index(1) != -1)
        {
            geometry_msgs::PoseStamped path_node;
            // path_node.pose.position.x = sweep_point_vec[current_index(0)][current_index(1)].point(0);
            // path_node.pose.position.y = sweep_point_vec[current_index(0)][current_index(1)].point(1);
            path_node.pose.position.x = pose.position.x;
            path_node.pose.position.y = pose.position.y;
            path.poses.push_back(path_node);
        }
        ROS_INFO("Sweep point vec after reFindLine");
        for(int i = 0;i < sweep_point_vec.size();i++)
        {
            for(int j = 0;j < sweep_point_vec[i].size();j++)
            {
                ROS_INFO("---line = %d,index = %d,x = %f,y = %f,status = %d",i,j+1,sweep_point_vec[i][j].point(0),sweep_point_vec[i][j].point(1),sweep_point_vec[i][j].visited);
            }
            ROS_INFO("-----------------------------------");
        }
    }
    else
    {
        sweepStatus = sweepPlan(path,pose,current_index,last_index);
        if(!path.poses.empty())    path_pub_.publish(path);
        return sweepStatus;
    }
    return false;
}

bool COVER_SWEEP::reFindLineByCol(Eigen::Vector2i& current_index,Eigen::Vector2i& last_index)
{
    sweep_point_vec[current_index(0)][current_index(1)].visited = false;
    Eigen::Vector2d last_point = sweep_point_vec[last_index(0)][last_index(1)].point;
    Eigen::Vector2i min_point(jps_plan_.world2grid(Eigen::Vector2d(border_min_x,border_min_y)));
    Eigen::Vector2i max_point(jps_plan_.world2grid(Eigen::Vector2d(border_max_x,border_max_y)));
    for(int i = 0;i < sweep_point_vec.size();i++)
    {
        if(isLineEmpty(sweep_point_vec[i]))     continue;
        std::vector<Eigen::Vector2d> candidate_point_vec;
        std::vector<SweepPoint> sweep_line_point;
        Eigen::Vector2i start_point(jps_plan_.world2grid(sweep_point_vec[i].front().point));
        Eigen::Vector2i end_point(jps_plan_.world2grid(sweep_point_vec[i].back().point));
        for(int j = start_point(1);j<= end_point(1);j++)
        {
            Eigen::Vector2i temp_point_grid(start_point(0),j);
            Eigen::Vector2i next_point_grid(start_point(0),j+1);
            int temp_map_index = jps_plan_.coord2Id(temp_point_grid);
            int next_map_index = jps_plan_.coord2Id(next_point_grid);
            if(expand_map_border.data[temp_map_index] != expand_map_border.data[next_map_index])
            {
                Eigen::Vector2d temp_point_world;
                if(expand_map_border.data[temp_map_index] == BORDER_VALUE && expand_map_border.data[next_map_index] == 0)
                {
                    temp_point_world = jps_plan_.grid2world(next_point_grid);
                    if((temp_point_world(0) <= border_min_x + 0.1) || (temp_point_world(0) >= border_max_x - 0.1))  continue;
                    candidate_point_vec.push_back(temp_point_world); 
                }
                else if(expand_map_border.data[next_map_index] == BORDER_VALUE && expand_map_border.data[temp_map_index] == 0)
                {
                    temp_point_world = jps_plan_.grid2world(temp_point_grid);
                    if((temp_point_world(0) <= border_min_x + 0.1) || (temp_point_world(0) >= border_max_x - 0.1))  continue;
                    candidate_point_vec.push_back(temp_point_world); 
                }
            }
        }
        //1、筛选符合条件的新增点
        std::vector<SweepPoint> new_sweep_point;
        for(int n = 0;n < sweep_point_vec[i].size() - 1;n++)
        {
            if(sweep_point_vec[i][n].visited && sweep_point_vec[i][n+1].visited)    continue;
            for(int m = 0;m < candidate_point_vec.size();m++)
            {
                if((candidate_point_vec[m](1) > (sweep_point_vec[i][n].point(1) - 0.1)) && 
                   (candidate_point_vec[m](1) < (sweep_point_vec[i][n+1].point(1) + 0.1)))
                {
                    if(fabs(candidate_point_vec[m](1) - sweep_point_vec[i][n].point(1)) < 0.1 ||
                       fabs(candidate_point_vec[m](1) - sweep_point_vec[i][n+1].point(1)) < 0.1)  continue;
                    SweepPoint temp_point  = {candidate_point_vec[m],false};
                    ROS_INFO("new point,index = %d,x = %f,y = %f",i,temp_point.point(0),temp_point.point(1));
                    new_sweep_point.push_back(temp_point);
                }
            }
        }
        //2、将新增的配对点插入到清扫队列中
        if(!new_sweep_point.empty())
        {
            for(int a = 0;a < new_sweep_point.size();a++)
            {
                for(int b = 0;b<sweep_point_vec[i].size() - 1;b++)
                {
                    if(new_sweep_point[a].point(1) > sweep_point_vec[i][b].point(1) &&
                       new_sweep_point[a].point(1) < sweep_point_vec[i][b+1].point(1))
                    {
                        sweep_point_vec[i].insert(sweep_point_vec[i].begin() + b + 1,new_sweep_point[a]);
                        break;
                    }
                }
            }
        }
        //3、清除更新后为占据状态的配对点
        for(int c = 0;c < sweep_point_vec[i].size();c++)
        {
            Eigen::Vector2i temp_point_grid = jps_plan_.world2grid(sweep_point_vec[i][c].point);
            if(jps_plan_.isOccupied(temp_point_grid,expand_map_border,BORDER_VALUE))
            {
                std::vector<SweepPoint>::iterator iter = sweep_point_vec[i].begin() + c;
                sweep_point_vec[i].erase(iter);
                c--;
            }      
        }
        //4、清除曲线环形之外的点
        if(!sweep_point_vec[i].empty())   
        {
            deleteOutCurvePointByCol(sweep_point_vec[i]);
        }
        //5、再将连通的点进行合并，筛选出配对点
        std::vector<SweepPoint> candidate_vec = sweep_point_vec[i];
        for(int p =0;p < candidate_vec.size();p++)
        {
            // if(candidate_vec[p].visited)    continue;
            for(int q = candidate_vec.size() - 1;q > p;q--)
            {
                // if(candidate_vec[q].visited)    continue;
                if(collisionCheck(candidate_vec[p].point,candidate_vec[q].point,border_map,BORDER_VALUE) && 
                    getEuclideanDistance(candidate_vec[p].point,candidate_vec[q].point) > 0.15)
                    {
                        SweepPoint start_point = candidate_vec[p];
                        SweepPoint goal_point  = candidate_vec[q];
                        sweep_line_point.push_back(start_point);
                        sweep_line_point.push_back(goal_point);
                        ROS_INFO("a pair point,point start x = %f,y = %f,point end x = %f,y = %f,line_index = %d",
                            start_point.point(0),start_point.point(1),goal_point.point(0),goal_point.point(1),i);
                        p = q;
                        break;
                    }
            }
        }
        sweep_point_vec[i].clear();
        sweep_point_vec[i].insert(sweep_point_vec[i].begin(),sweep_line_point.begin(),sweep_line_point.end());

    }
    //更新current_index
    if(current_index(0) == last_index(0))
    {
        for(int k = 0;k < sweep_point_vec[last_index(0)].size();k++)
        {
            if(sweep_point_vec[last_index(0)][k].point == last_point)
            {
                last_index << last_index(0),k;
                ROS_INFO("last point update,index x = %d,y = %d",last_index(0),last_index(1));
                if(current_index(1) > last_index(1))    current_index << current_index(0),last_index(1) + 1;
                else                                    current_index << current_index(0),last_index(1) - 1;
            }
        }
        if(sweep_point_vec[last_index(0)][last_index(1)].point != last_point)
        {
            current_index << -1,-1;
        }
        else
        {
            ROS_INFO("new current index x = %d,y = %d",current_index(0),current_index(1));
            sweep_point_vec[current_index(0)][current_index(1)].visited = true;
        }
    }
    else
    {
        current_index << -1,-1;
    }
    return true;
}

bool COVER_SWEEP::find_lineByCol()
{
    Eigen::Vector2i min_point(jps_plan_.world2grid(Eigen::Vector2d(border_min_x,border_min_y)));
    Eigen::Vector2i max_point(jps_plan_.world2grid(Eigen::Vector2d(border_max_x,border_max_y)));
    int line_index = 1;
    for(int x = min_point(0);x < max_point(0);)
    {
        std::vector<Eigen::Vector2d> candidate_point_vec;
        std::vector<SweepPoint> sweep_line_point;
        for(int y = min_point(1);y < max_point(1) - 1;y++)
        {
            Eigen::Vector2i temp_point_grid(x,y);
            Eigen::Vector2i next_point_grid(x,y+1);
            int temp_map_index = jps_plan_.coord2Id(temp_point_grid);
            int next_map_index = jps_plan_.coord2Id(next_point_grid);
            if(expand_map_border.data[temp_map_index] != expand_map_border.data[next_map_index])
            {
                Eigen::Vector2d temp_point_world;
                if(expand_map_border.data[temp_map_index] == BORDER_VALUE && expand_map_border.data[next_map_index] == 0)
                {
                    temp_point_world = jps_plan_.grid2world(next_point_grid);
                    if((temp_point_world(0) <= border_min_x + 0.1) || (temp_point_world(0) >= border_max_x - 0.1))  continue;
                    // ROS_INFO("candidate point x = %f,y = %f",temp_point_world(0),temp_point_world(1));
                    candidate_point_vec.push_back(temp_point_world); 
                }
                else if(expand_map_border.data[next_map_index] == BORDER_VALUE && expand_map_border.data[temp_map_index] == 0)
                {
                    temp_point_world = jps_plan_.grid2world(temp_point_grid);
                    if((temp_point_world(0) <= border_min_x + 0.1) || (temp_point_world(0) >= border_max_x - 0.1))  continue;
                    // ROS_INFO("candidate point x = %f,y = %f",temp_point_world(0),temp_point_world(1));
                    candidate_point_vec.push_back(temp_point_world); 
                }
            }
        }
        if(candidate_point_vec.size() < 2)      {x++;}
        else
        {
            //遍历候选点集合，筛选符合条件的配对点
            for(int i =0;i < candidate_point_vec.size();i++)
            {
                for(int j = candidate_point_vec.size() - 1;j > i;j--)
                {
                    if(collisionCheck(candidate_point_vec[i],candidate_point_vec[j],border_map,BORDER_VALUE) && 
                       getEuclideanDistance(candidate_point_vec[i],candidate_point_vec[j]) > 0.15)
                       {
                            SweepPoint start_point = {candidate_point_vec[i],false};
                            SweepPoint goal_point  = {candidate_point_vec[j],false};
                            sweep_line_point.push_back(start_point);
                            sweep_line_point.push_back(goal_point);
                            ROS_INFO("a pair point,point start x = %f,y = %f,point end x = %f,y = %f,line_index = %d",
                              start_point.point(0),start_point.point(1),goal_point.point(0),goal_point.point(1),line_index);
                            i = j;
                            break;
                       }
                }
            }
            
            //判断配对点中是否存在有沿边曲线外的，若有，则删除
            if(!sweep_line_point.empty())   
            {
                int origin_size = sweep_line_point.size();
                deleteOutCurvePointByCol(sweep_line_point);
            }

            //遍历完候选点集合，判断当前线上是否有满足条件的配对点。若无，则搜索下一列，反之，则搜索一个GAP宽的下一列
            if(!sweep_line_point.empty()) 
            {
                sweep_point_vec.push_back(sweep_line_point);
                x = x + (int)(0.25 / expand_map_border.info.resolution);
                line_index++;
            }  
            else
            {
                x++;
            }
        }
    }
    // ROS_INFO("sweep_point_vec size = %d\n",sweep_point_vec.size());
    // if(!sweep_point_vec.empty())     return true;
    // return false;
    return true;
}

void COVER_SWEEP::deleteOutCurvePointByCol(std::vector<SweepPoint>& sweep_line_point)
{
    for(int i = 0;i < sweep_line_point.size();i++)
    {
        Eigen::Vector2i current_point_grid(jps_plan_.world2grid(Eigen::Vector2d(sweep_line_point[i].point(0),sweep_line_point[i].point(1))));
        bool search_up_result = isOutCruvePointByCol(current_point_grid,1);
        bool search_down_result = isOutCruvePointByCol(current_point_grid,-1);
        if(search_up_result && search_down_result)
        {
            // ROS_INFO("point x = %f,y = %f is out of cruve!!!",sweep_line_point[i].point(0),sweep_line_point[i].point(1));
            auto iter = sweep_line_point.begin() + i;
            sweep_line_point.erase(iter);
            i--;
        }
        else if(search_up_result || search_down_result)
        {
            bool search_left_result = isOutCruvePointByRow(current_point_grid,1);
            bool search_right_result = isOutCruvePointByRow(current_point_grid,-1);
            if(search_left_result || search_right_result)
            {
                // ROS_INFO("point x = %f,y = %f is out of cruve by double check!!!",sweep_line_point[i].point(0),sweep_line_point[i].point(1));
                auto iter = sweep_line_point.begin() + i;
                sweep_line_point.erase(iter);
                i--;
            }
        }
    }
}

bool COVER_SWEEP::sweepPlan(nav_msgs::Path& path,geometry_msgs::Pose pose,Eigen::Vector2i& current_index,Eigen::Vector2i& last_index)
{
    if(path.poses.size() >=2)
    {
        if(!(fabs(pose.position.x - path.poses.back().pose.position.x) < 0.1 &&
            fabs(pose.position.y - path.poses.back().pose.position.y) < 0.1) )                return false;
        else
        {
            path.poses.erase(path.poses.begin(),path.poses.begin() + path.poses.size() -1);
        }
    }

    bool findPoint_status = false;
    if(current_index(0) == -1 && current_index(1) == -1)
    {
        current_index = findInitPoint(pose);
        ROS_INFO("init sweep index: x = %d,y = %d",current_index(0),current_index(1));
        sweep_point_vec[current_index(0)][current_index(1)].visited = true;
        Eigen::Vector2d start_pose(pose.position.x,pose.position.y);
        if(collisionCheck(start_pose,sweep_point_vec[current_index(0)][current_index(1)].point,border_map,BORDER_VALUE))
        {
            ROS_INFO("init to point pass!");
            geometry_msgs::PoseStamped path_node;
            path_node.pose.position.x = pose.position.x;
            path_node.pose.position.y = pose.position.y;
            path.poses.push_back(path_node);
            path_node.pose.position.x = sweep_point_vec[current_index(0)][current_index(1)].point(0);
            path_node.pose.position.y = sweep_point_vec[current_index(0)][current_index(1)].point(1);
            path.poses.push_back(path_node);
        }
        else
        {
            ROS_INFO("init to point not pass!");
            if(jps_plan_.globalPathPlaning(start_pose,sweep_point_vec[current_index(0)][current_index(1)].point,expand_map))
            {
                ROS_INFO("jps search success");
                nav_msgs::Path temp_path = jps_plan_.getGlobalPath();
                path.poses.insert(path.poses.begin(),temp_path.poses.begin(),temp_path.poses.end());
            }
            else    {ROS_INFO("jps search fail");}
        }     
       //起始位置偏右，则主方向为负;反之，起始位置偏左，则主方向为正
       if(main_direction == 0)
            main_direction = current_index(0) > (sweep_point_vec.size() / 2) ? -1 : 1;     
        ROS_INFO("Init point index x = %d,y = %d,main direction = %d",current_index(0),current_index(1),main_direction);
        findPoint_status = find_next_point(current_index,path,last_index);
        // if(findPoint_status)    ROS_INFO("sweep index: x = %d,y = %d",current_index(0),current_index(1));
        return !findPoint_status;
    }
    else
    {
        findPoint_status = find_next_point(current_index,path,last_index);
        // if(findPoint_status)    ROS_INFO("sweep index: x = %d,y = %d",current_index(0),current_index(1));
        // return !findPoint_status;               //找下一个点失败，则路径规划完成，返回true
        if(!findPoint_status)   return true;
        if(findPoint_status)    
        {
            findPoint_status = find_next_point(current_index,path,last_index);
            // if(findPoint_status)    ROS_INFO("sweep index: x = %d,y = %d",current_index(0),current_index(1));
            return !findPoint_status;               //找下一个点失败，则路径规划完成，返回true
        }
        else
        {
            return true;
        }
    }
}

bool COVER_SWEEP::find_next_point(Eigen::Vector2i& current_index,nav_msgs::Path& path,Eigen::Vector2i& last_index)
{
    // ROS_INFO("!!!!!!!!!----current index x = %d,y = %d,prepare find next point---!!!!!!!!",current_index(0),current_index(1));
    Eigen::Vector2i next_index;
    nav_msgs::Path jps_path;
    //第一步：通过直线找当前直线上是否有满足配对的点
    // ROS_INFO("run to here 0");
    if(findCurrentLineWithStraight(current_index,next_index))
    {
        // ROS_INFO("run to here 0");
        ROS_INFO("run to here 0,next_index x,y = %d,%d",next_index(0),next_index(1));
        sweep_point_vec[next_index(0)][next_index(1)].visited = true;
        last_index = current_index;
        current_index = next_index;
        geometry_msgs::PoseStamped path_node;
        path_node.pose.position.x = sweep_point_vec[current_index(0)][current_index(1)].point(0);
        path_node.pose.position.y = sweep_point_vec[current_index(0)][current_index(1)].point(1);
        path.poses.push_back(path_node);
        return true;
    }
    //第二步：通过直线找附近直线上是否有满足配对的点
    else if(findNearLineWithStraight(current_index,next_index))
    {
        ROS_INFO("run to here 1,next_index x,y = %d,%d",next_index(0),next_index(1));
        // ROS_INFO("run to here 1");
        sweep_point_vec[next_index(0)][next_index(1)].visited = true;
        last_index = current_index;
        current_index = next_index;
        geometry_msgs::PoseStamped path_node;
        path_node.pose.position.x = sweep_point_vec[current_index(0)][current_index(1)].point(0);
        path_node.pose.position.y = sweep_point_vec[current_index(0)][current_index(1)].point(1);
        path.poses.push_back(path_node);
        return true;
    }
    //第三步：通过路径规划找当前直线上是否有满足配对点
    else if(findCurrentLineWithJPS(current_index,next_index,jps_path))
    {
        ROS_INFO("run to here 2,next_index x,y = %d,%d",next_index(0),next_index(1));
        // ROS_INFO("run to here 2");
        sweep_point_vec[next_index(0)][next_index(1)].visited = true;
        last_index = current_index;
        current_index = next_index;
        path.poses.insert(path.poses.end(),jps_path.poses.begin(),jps_path.poses.end());
        return true;
    }
    //第四步：通过路径规划找附近直线上是否有满足配对的点
    else if(findNearLineWithJPS(current_index,next_index,jps_path))
    {
        ROS_INFO("run to here 3,next_index x,y = %d,%d",next_index(0),next_index(1));
        // ROS_INFO("run to here 3");
        sweep_point_vec[next_index(0)][next_index(1)].visited = true;
        last_index = current_index;
        current_index = next_index;
        path.poses.insert(path.poses.end(),jps_path.poses.begin(),jps_path.poses.end());
        return true;
    }
    else
    {
        ROS_INFO("cover sweep finished");
        current_index << -1,-1;
        last_index << -1,-1;
        for(int i = 0;i < sweep_point_vec.size();i++)
        {
            for(int j = 0;j < sweep_point_vec[i].size();j++)
            {
                if(!sweep_point_vec[i][j].visited)
                ROS_INFO("---line = %d,index = %d,x = %f,y = %f,status = %d",i,j+1,sweep_point_vec[i][j].point(0),sweep_point_vec[i][j].point(1),sweep_point_vec[i][j].visited);
            }
        }
        return false;               //搜索点下一个点失败，则弓扫规划结束
    }
}

bool COVER_SWEEP::findCurrentLineWithStraight(const Eigen::Vector2i current_index,Eigen::Vector2i& next_index)
{
    //先向下搜索，再向上搜索
    if(findCurrentLineWithStraight(current_index,next_index,-1))        return true;
    else if(findCurrentLineWithStraight(current_index,next_index,1))    return true;
    else                                                    return false;
}

bool COVER_SWEEP::findCurrentLineWithStraight(const Eigen::Vector2i current_index,Eigen::Vector2i& next_index,int direction)
{
    int next_y = current_index(1) + direction;  //向上，direction = 1;向下，direction = -1
    if(direction == -1 && next_y < 0)   return false;
    if(direction == 1  && next_y >= sweep_point_vec[current_index(0)].size())       return false;
    if(sweep_point_vec[current_index(0)][next_y].visited)    return false;
    Eigen::Vector2d current_point(sweep_point_vec[current_index(0)][current_index(1)].point);
    Eigen::Vector2d next_point(sweep_point_vec[current_index(0)][next_y].point);
    if(collisionCheck(current_point,next_point,border_map,BORDER_VALUE))
    {
        next_index << current_index(0),next_y;
        return true;
    } 
    return false;  
}

bool COVER_SWEEP::findNearLineWithStraight(const Eigen::Vector2i current_index,Eigen::Vector2i& next_index)
{
    float min_distance_left = 100,min_distance_right = 100;
    bool find_left_status = false,find_right_status = false;
    Eigen::Vector2i next_index_left,next_index_right;
    //定义：序号减小的方向就是左，反之为右
    find_left_status =  findNearLineWithStraight(current_index,next_index_left,-1,min_distance_left);
    find_right_status = findNearLineWithStraight(current_index,next_index_right,1,min_distance_right);
    // ROS_INFO("left status = %d,right status = %d",find_left_status,find_right_status);
    if(find_left_status && !find_right_status)
    {
        next_index = next_index_left;
        return true;
    }
    else if(!find_left_status && find_right_status)
    {
        next_index = next_index_right;
        return true;
    }
    else if(find_left_status && find_right_status)
    {
        // ROS_INFO("left dis = %f,right dis = %f",min_distance_left,min_distance_right);
        if(min_distance_left < min_distance_right * 2.0)
        {
            //主方向为1,则向左补扫，优先向左搜索
            if(main_direction == 1)         next_index = next_index_left;
            else if(main_direction == -1)   next_index = next_index_right;
            return true;
        }
        else 
        {
            next_index = next_index_right;
            return true;
        }
    }
    else
    {
        return false;
    }
}

bool COVER_SWEEP::findNearLineWithStraight(const Eigen::Vector2i current_index,Eigen::Vector2i& next_index,int direction,float& min_distance)
{
    for(int i = current_index(0);i >=0 && i <sweep_point_vec.size();i=i+direction )
    {
        if(i == current_index(0))      continue;

            Eigen::Vector2i min_distance_index;
            for(int j = 0;j < sweep_point_vec[i].size();j++)
            {
                if(sweep_point_vec[i][j].visited)       continue;
                Eigen::Vector2d current_point(sweep_point_vec[current_index(0)][current_index(1)].point);
                Eigen::Vector2d next_point(sweep_point_vec[i][j].point); 
                if(collisionCheck(current_point,next_point,border_map,BORDER_VALUE))
                {
                    float distance = getEuclideanDistance(current_point,next_point);
                    if(distance < min_distance)
                    {
                        min_distance = distance;
                        min_distance_index << i,j;
                    }
                }
            }
            if(min_distance != 100)
            {
                if(sweepByCol)      //竖向清扫，需要剔除的直线找其它列的结果
                {
                    float diff_x = sweep_point_vec[current_index(0)][current_index(1)].point(0) -
                                sweep_point_vec[min_distance_index(0)][min_distance_index(1)].point(0);
                    if(fabs(current_index(0) - min_distance_index(0)) > 4)  return false;
                    if((min_distance > 3.0 * fabs(diff_x)) && fabs(current_index(0) - min_distance_index(0))>2)   return false;
                }
                else                //横向清扫，需要剔除的直线找其它列的结果
                {
                    float diff_y = sweep_point_vec[current_index(0)][current_index(1)].point(1) -
                                sweep_point_vec[min_distance_index(0)][min_distance_index(1)].point(1);
                    if(fabs(current_index(0) - min_distance_index(0)) > 4)  return false;
                    if((min_distance > 3.0 * fabs(diff_y)) && fabs(fabs(current_index(0) - min_distance_index(0)) > 2))   return false;                    
                }

                if(!isVaildLine(current_index,min_distance_index))   return false;
                next_index = min_distance_index;
                return true;
            }
    }
    return false;
}

bool COVER_SWEEP::findCurrentLineWithJPS(const Eigen::Vector2i current_index,
                                        Eigen::Vector2i& next_index,nav_msgs::Path& path)
{
    float min_distance_up = 100,min_distance_down = 100;
    bool find_up_status = false,find_down_status = false;
    Eigen::Vector2i next_index_up,next_index_down;
    nav_msgs::Path jps_path_up,jps_path_down;
    find_up_status = findCurrentLineWithJPS(current_index,next_index_up,jps_path_up,1,min_distance_up);
    find_down_status = findCurrentLineWithJPS(current_index,next_index_down,jps_path_down,-1,min_distance_down);
    if(find_up_status && !find_down_status)
    {
        next_index = next_index_up;
        path = jps_path_up;
        return true;
    }
    else if(!find_up_status && find_down_status)
    {
        next_index = next_index_down;
        path = jps_path_down;
        return true;
    }
    else if(find_up_status && find_down_status)
    {
        if(min_distance_up <= min_distance_down)
        {
            next_index = next_index_up;
            path = jps_path_up;
        }
        else
        {
            next_index = next_index_down;
            path = jps_path_down;
        }
        return true;
    }
    else if(!find_up_status && !find_down_status)
    {
        return false;
    }
}

bool COVER_SWEEP::findCurrentLineWithJPS(const Eigen::Vector2i current_index,Eigen::Vector2i& next_index,
                                         nav_msgs::Path& path,int direction,float& min_distance)
{
    for(int i = current_index(1);i >= 0 && i < sweep_point_vec[current_index(0)].size();i = i + direction)
    {
        if(i == current_index(1))       continue;
        if(sweep_point_vec[current_index(0)][i].visited)    continue;
        Eigen::Vector2d current_point(sweep_point_vec[current_index(0)][current_index(1)].point);
        Eigen::Vector2d next_point(sweep_point_vec[current_index(0)][i].point);
        float distance = getJpsDistance(current_point,next_point);
        if(distance != 1000)
        {
            next_index << current_index(0),i;
            path = jps_plan_.getGlobalPath();
            min_distance = distance;
            return true;
        }
    }
    
    return false;
}

bool COVER_SWEEP::findNearLineWithJPS(const Eigen::Vector2i current_index,
                                      Eigen::Vector2i& next_index,nav_msgs::Path& path)
{
    float min_distance_left = 100,min_distance_right = 100;
    bool find_left_status = false,find_right_status = false;
    Eigen::Vector2i next_index_left,next_index_right;
    nav_msgs::Path path_left,path_right;
    find_left_status = findNearLineWithJPS(current_index,next_index_left,path_left,-1,min_distance_left);
    find_right_status =findNearLineWithJPS(current_index,next_index_right,path_right,1,min_distance_right);
    if(find_left_status && !find_right_status)
    {
        next_index = next_index_left;
        path = path_left;
        return true;
    }
    else if(!find_left_status && find_right_status)
    {
        next_index = next_index_right;
        path = path_right;
        return true;
    }
    else if(find_left_status && find_right_status)
    {
        if(min_distance_left < min_distance_right * 2.0)
        {
            //主方向为1,则向左补扫，优先向左搜索
            if(main_direction == 1)
            {
                next_index = next_index_left;
                path = path_left;
            }
            else if(main_direction == -1)
            {
                next_index = next_index_right;
                path = path_right;  
            }

        }
        else 
        {
            next_index = next_index_right;
            path = path_right;
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool COVER_SWEEP::findNearLineWithJPS(const Eigen::Vector2i current_index,
                                      Eigen::Vector2i& next_index,nav_msgs::Path& path,int direction,float& min_distance)
{
    for(int i = current_index(0);i >=0 && i < sweep_point_vec.size();i = i + direction)
    {
        if(i == current_index(0))   continue;
        Eigen::Vector2i min_distance_index;
        nav_msgs::Path temp_path;
        for(int j = 0;j < sweep_point_vec[i].size();j++)
        {
            if(sweep_point_vec[i][j].visited)   continue;
            Eigen::Vector2d current_point(sweep_point_vec[current_index(0)][current_index(1)].point);
            Eigen::Vector2d next_point(sweep_point_vec[i][j].point);
            float distance = getJpsDistance(current_point,next_point);
            if(distance != 1000)
            {
                if(distance < min_distance)
                {
                    min_distance = distance;
                    min_distance_index << i,j;
                    temp_path = jps_plan_.getGlobalPath();
                }
            }
        }
        if(min_distance != 100)
        {
            next_index = min_distance_index;
            path = temp_path;
            return true;
        }
    }
    return false;
}



/*纵向清扫算法：搜索配对点、更新配对点、规划路径*/
bool COVER_SWEEP::coverPlanByRow(nav_msgs::Path& path,geometry_msgs::Pose pose)
{
    static Eigen::Vector2i current_index(-1,-1);
    static Eigen::Vector2i last_index(-1,-1);
    if(!findLineStatus)
    {  
        findLineStatus = find_lineByRow();
        ROS_INFO("Sweep point vec after init find line:");
        for(int i = 0;i < sweep_point_vec.size();i++)
        {
            for(int j = 0;j < sweep_point_vec[i].size();j++)
            {
                ROS_INFO("line = %d,index = %d,x = %f,y = %f,status = %d",i,j+1,sweep_point_vec[i][j].point(0),sweep_point_vec[i][j].point(1),sweep_point_vec[i][j].visited);
            }
            ROS_INFO("-----------------------------------");
        }
        /*int direction = 0;
        for(int i = 0;i < sweep_point_vec.size();i++)
        {
            ROS_INFO("i = %d,size = %d",i,sweep_point_vec[i].size());
            if(i %2 == 0)
            {
                for(int j = 0;j < sweep_point_vec[i].size();j++)
                {
                    ROS_INFO("line = %d,index = %d,x = %f,y = %f,status = %d",i,j,sweep_point_vec[i][j].point(0),sweep_point_vec[i][j].point(1),sweep_point_vec[i][j].visited);
                    geometry_msgs::PoseStamped temp;
                    temp.pose.position.x = sweep_point_vec[i][j].point(0);
                    temp.pose.position.y = sweep_point_vec[i][j].point(1);
                    path.poses.push_back(temp);
                }
            }
            else
            {
                for(int j = sweep_point_vec[i].size() - 1;j >=0;j--)
                {
                    geometry_msgs::PoseStamped temp;
                    temp.pose.position.x = sweep_point_vec[i][j].point(0);
                    temp.pose.position.y = sweep_point_vec[i][j].point(1);
                    path.poses.push_back(temp);
                }
            }
        }
        if(!path.poses.empty())    path_pub_.publish(path);
        return true;*/
    }
    else if(!updateLineStatus)         //更新sweep_point_vec,并更新current_index 
    {
        updateLineStatus = reFindLineByRow(current_index,last_index);
        if(current_index(0) != -1 && current_index(1) != -1)
        {
            geometry_msgs::PoseStamped path_node;
            path_node.pose.position.x = pose.position.x;
            path_node.pose.position.y = pose.position.y;
            path.poses.push_back(path_node);
        }
        ROS_INFO("Sweep point vec after reFindLine");
        for(int i = 0;i < sweep_point_vec.size();i++)
        {
            for(int j = 0;j < sweep_point_vec[i].size();j++)
            {
                ROS_INFO("---line = %d,index = %d,x = %f,y = %f,status = %d",i,j+1,sweep_point_vec[i][j].point(0),sweep_point_vec[i][j].point(1),sweep_point_vec[i][j].visited);
            }
            ROS_INFO("-----------------------------------");
        }
    }
    else
    {
        sweepStatus = sweepPlan(path,pose,current_index,last_index);
        if(!path.poses.empty())    path_pub_.publish(path);
        return sweepStatus;
    }
    return false;
}

bool COVER_SWEEP::reFindLineByRow(Eigen::Vector2i& current_index,Eigen::Vector2i& last_index)
{
    sweep_point_vec[current_index(0)][current_index(1)].visited = false;
    Eigen::Vector2d last_point = sweep_point_vec[last_index(0)][last_index(1)].point;
    Eigen::Vector2i min_point(jps_plan_.world2grid(Eigen::Vector2d(border_min_x,border_min_y)));
    Eigen::Vector2i max_point(jps_plan_.world2grid(Eigen::Vector2d(border_max_x,border_max_y)));
    for(int i = 0;i < sweep_point_vec.size();i++)
    {
        if(isLineEmpty(sweep_point_vec[i]))     continue;
        std::vector<Eigen::Vector2d> candidate_point_vec;
        std::vector<SweepPoint> sweep_line_point;
        Eigen::Vector2i start_point(jps_plan_.world2grid(sweep_point_vec[i].front().point));
        Eigen::Vector2i end_point(jps_plan_.world2grid(sweep_point_vec[i].back().point));
        for(int j = start_point(0);j<= end_point(0);)
        {
            Eigen::Vector2i temp_point_grid(j,start_point(1));
            Eigen::Vector2i next_point_grid(j+1,start_point(1));
            int temp_map_index = jps_plan_.coord2Id(temp_point_grid);
            int next_map_index = jps_plan_.coord2Id(next_point_grid);
            if(expand_map_border.data[temp_map_index] != expand_map_border.data[next_map_index])
            {
                Eigen::Vector2d temp_point_world;
                if(expand_map_border.data[temp_map_index] == BORDER_VALUE && expand_map_border.data[next_map_index] == 0)
                {
                    temp_point_world = jps_plan_.grid2world(next_point_grid);
                    if((temp_point_world(1) <= border_min_y + 0.1) || (temp_point_world(1) >= border_max_y - 0.1))  continue;
                    candidate_point_vec.push_back(temp_point_world);
                    // ROS_INFO("candidata point x = %f,y = %f",temp_point_world(0),temp_point_world(1)); 
                }
                else if(expand_map_border.data[next_map_index] == BORDER_VALUE && expand_map_border.data[temp_map_index] == 0)
                {
                    temp_point_world = jps_plan_.grid2world(temp_point_grid);
                    if((temp_point_world(1) <= border_min_y + 0.1) || (temp_point_world(1) >= border_max_y - 0.1))  continue;
                    candidate_point_vec.push_back(temp_point_world);
                    // ROS_INFO("candidata point x = %f,y = %f",temp_point_world(0),temp_point_world(1)); 
                }
                j += 2;
            }
            j++;
        }
        //1、筛选符合条件的新增点
        std::vector<SweepPoint> new_sweep_point;
        for(int n = 0;n < sweep_point_vec[i].size() - 1;n++)
        {
            if(sweep_point_vec[i][n].visited && sweep_point_vec[i][n+1].visited)    continue;
            for(int m = 0;m < candidate_point_vec.size();m++)
            {
                if((candidate_point_vec[m](0) > (sweep_point_vec[i][n].point(0) - 0.1)) && 
                   (candidate_point_vec[m](0) < (sweep_point_vec[i][n+1].point(0) + 0.1)))
                {
                    if(fabs(candidate_point_vec[m](0) - sweep_point_vec[i][n].point(0)) < 0.1 ||
                       fabs(candidate_point_vec[m](0) - sweep_point_vec[i][n+1].point(0)) < 0.1)  continue;
                    SweepPoint temp_point  = {candidate_point_vec[m],false};
                    ROS_INFO("new point,index = %d,x = %f,y = %f",i,temp_point.point(0),temp_point.point(1));
                    new_sweep_point.push_back(temp_point);
                }
            }
        }
        //2、将新增的配对点插入到清扫队列中
        if(!new_sweep_point.empty())
        {
            for(int a = 0;a < new_sweep_point.size();a++)
            {
                for(int b = 0;b<sweep_point_vec[i].size() - 1;b++)
                {
                    if(new_sweep_point[a].point(0) > sweep_point_vec[i][b].point(0) &&
                       new_sweep_point[a].point(0) < sweep_point_vec[i][b+1].point(0))
                    {
                        sweep_point_vec[i].insert(sweep_point_vec[i].begin() + b + 1,new_sweep_point[a]);
                        break;
                    }
                }
            }
        }
        //3、清除更新后为占据状态的配对点
        for(int c = 0;c < sweep_point_vec[i].size();c++)
        {
            Eigen::Vector2i temp_point_grid = jps_plan_.world2grid(sweep_point_vec[i][c].point);
            if(jps_plan_.isOccupied(temp_point_grid,expand_map_border,BORDER_VALUE))
            {
                std::vector<SweepPoint>::iterator iter = sweep_point_vec[i].begin() + c;
                sweep_point_vec[i].erase(iter);
                c--;
            }      
        }
        //4、清除曲线环形之外的点
        if(!sweep_point_vec[i].empty())   
        {
            deleteOutCurvePointByRow(sweep_point_vec[i]);
        }
        //5、再将连通的点进行合并，筛选出配对点
        std::vector<SweepPoint> candidate_vec = sweep_point_vec[i];
        for(int p =0;p < candidate_vec.size();p++)
        {
            // if(candidate_vec[p].visited)    continue;
            for(int q = candidate_vec.size() - 1;q > p;q--)
            {
                // if(candidate_vec[q].visited)    continue;
                if(collisionCheck(candidate_vec[p].point,candidate_vec[q].point,border_map,BORDER_VALUE) && 
                    getEuclideanDistance(candidate_vec[p].point,candidate_vec[q].point) > 0.15)
                    {
                        SweepPoint start_point = candidate_vec[p];
                        SweepPoint goal_point  = candidate_vec[q];
                        sweep_line_point.push_back(start_point);
                        sweep_line_point.push_back(goal_point);
                        ROS_INFO("a pair point,point start x = %f,y = %f,point end x = %f,y = %f,line_index = %d",
                            start_point.point(0),start_point.point(1),goal_point.point(0),goal_point.point(1),i);
                        p = q;
                        break;
                    }
            }
        }
        sweep_point_vec[i].clear();
        sweep_point_vec[i].insert(sweep_point_vec[i].begin(),sweep_line_point.begin(),sweep_line_point.end());

    }
    //更新current_index
    if(current_index(0) == last_index(0))
    {
        for(int k = 0;k < sweep_point_vec[last_index(0)].size();k++)
        {
            if(sweep_point_vec[last_index(0)][k].point == last_point)
            {
                last_index << last_index(0),k;
                ROS_INFO("last point update,index x = %d,y = %d",last_index(0),last_index(1));
                if(current_index(1) > last_index(1))    current_index << current_index(0),last_index(1) + 1;
                else                                    current_index << current_index(0),last_index(1) - 1;
            }
        }
        if(sweep_point_vec[last_index(0)][last_index(1)].point != last_point)
        {
            current_index << -1,-1;
        }
        else
        {
            ROS_INFO("new current index x = %d,y = %d",current_index(0),current_index(1));
            sweep_point_vec[current_index(0)][current_index(1)].visited = true;
        }
    }
    else
    {
        current_index << -1,-1;
    }
    return true;
}

bool COVER_SWEEP::find_lineByRow()
{
    Eigen::Vector2i min_point(jps_plan_.world2grid(Eigen::Vector2d(border_min_x,border_min_y)));
    Eigen::Vector2i max_point(jps_plan_.world2grid(Eigen::Vector2d(border_max_x,border_max_y)));
    int line_index = 1;
    for(int y = min_point(1);y < max_point(1);)
    {
        std::vector<Eigen::Vector2d> candidate_point_vec;
        std::vector<SweepPoint> sweep_line_point;
        for(int x = min_point(0);x < max_point(0) - 1;x++)
        {
            Eigen::Vector2i temp_point_grid(x,y);
            Eigen::Vector2i next_point_grid(x+1,y);
            int temp_map_index = jps_plan_.coord2Id(temp_point_grid);
            int next_map_index = jps_plan_.coord2Id(next_point_grid);
            if(expand_map_border.data[temp_map_index] != expand_map_border.data[next_map_index])
            {
                Eigen::Vector2d temp_point_world;
                if(expand_map_border.data[temp_map_index] == BORDER_VALUE && expand_map_border.data[next_map_index] == 0)
                {
                    temp_point_world = jps_plan_.grid2world(next_point_grid);
                    if((temp_point_world(1) <= border_min_y + 0.1) || (temp_point_world(1) >= border_max_y - 0.1))  continue;
                    // ROS_INFO("candidate point x = %f,y = %f",temp_point_world(0),temp_point_world(1));
                    candidate_point_vec.push_back(temp_point_world); 
                }
                else if(expand_map_border.data[next_map_index] == BORDER_VALUE && expand_map_border.data[temp_map_index] == 0)
                {
                    temp_point_world = jps_plan_.grid2world(temp_point_grid);
                    if((temp_point_world(1) <= border_min_y + 0.1) || (temp_point_world(1) >= border_max_y - 0.1))  continue;
                    // ROS_INFO("candidate point x = %f,y = %f",temp_point_world(0),temp_point_world(1));
                    candidate_point_vec.push_back(temp_point_world); 
                }
            }
        }
        if(candidate_point_vec.size() < 2)      {y++;}
        else
        {
            //遍历候选点集合，筛选符合条件的配对点
            for(int i =0;i < candidate_point_vec.size();i++)
            {
                for(int j = candidate_point_vec.size() - 1;j > i;j--)
                {
                    if(collisionCheck(candidate_point_vec[i],candidate_point_vec[j],border_map,BORDER_VALUE) && 
                       getEuclideanDistance(candidate_point_vec[i],candidate_point_vec[j]) > 0.15)
                       {
                            SweepPoint start_point = {candidate_point_vec[i],false};
                            SweepPoint goal_point  = {candidate_point_vec[j],false};
                            sweep_line_point.push_back(start_point);
                            sweep_line_point.push_back(goal_point);
                            ROS_INFO("a pair point,point start x = %f,y = %f,point end x = %f,y = %f,line_index = %d",
                              start_point.point(0),start_point.point(1),goal_point.point(0),goal_point.point(1),line_index);
                            i = j;
                            break;
                       }
                }
            }
            
            //判断配对点中是否存在有沿边曲线外的，若有，则删除
            //TODO:将纵向规划的去环外点改为横向
            if(!sweep_line_point.empty())   
            {
                int origin_size = sweep_line_point.size();
                deleteOutCurvePointByRow(sweep_line_point);
                // ROS_INFO("origin size = %d,current size = %d",origin_size,sweep_line_point.size());
            }

            //遍历完候选点集合，判断当前线上是否有满足条件的配对点。若无，则搜索下一列，反之，则搜索一个GAP宽的下一列
            if(!sweep_line_point.empty()) 
            {
                sweep_point_vec.push_back(sweep_line_point);
                y = y + (int)(0.25 / expand_map_border.info.resolution);
                line_index++;
            }  
            else
            {
                y++;
            }
        }
    }
    // ROS_INFO("sweep_point_vec size = %d\n",sweep_point_vec.size());
    // if(!sweep_point_vec.empty())     return true;
    // return false;
    return true;
}

void COVER_SWEEP::deleteOutCurvePointByRow(std::vector<SweepPoint>& sweep_line_point)
{
    for(int i = 0;i < sweep_line_point.size();i++)
    {
        Eigen::Vector2i current_point_grid(jps_plan_.world2grid(Eigen::Vector2d(sweep_line_point[i].point(0),sweep_line_point[i].point(1))));
        bool search_left_result = isOutCruvePointByRow(current_point_grid,1);
        bool search_right_result = isOutCruvePointByRow(current_point_grid,-1);       

        if(search_left_result && search_right_result)
        {
            // ROS_INFO("point x = %f,y = %f is out of cruve!!!",sweep_line_point[i].point(0),sweep_line_point[i].point(1));
            auto iter = sweep_line_point.begin() + i;
            sweep_line_point.erase(iter);
            i--;
        }
        else if(search_left_result || search_right_result)
        {
            bool search_up_result = isOutCruvePointByCol(current_point_grid,1);
            bool search_down_result = isOutCruvePointByCol(current_point_grid,-1);
            if(search_up_result || search_down_result)
            {
                // ROS_INFO("point x = %f,y = %f is out of cruve by double check!!!",sweep_line_point[i].point(0),sweep_line_point[i].point(1));
                auto iter = sweep_line_point.begin() + i;
                sweep_line_point.erase(iter);
                i--;
            }
            else
            {
                // ROS_INFO("point x = %f,y = %f is inner of cruve by double check----",sweep_line_point[i].point(0),sweep_line_point[i].point(1));
            }
        }
        else
        {
            // ROS_INFO("point x = %f,y = %f is inner of cruve----",sweep_line_point[i].point(0),sweep_line_point[i].point(1));
        }
    }
}





//竖向检查是否为沿边曲线外的点
bool COVER_SWEEP::isOutCruvePointByCol(const Eigen::Vector2i point_grid,const int direction)  
{
    Eigen::Vector2i min_point(jps_plan_.world2grid(Eigen::Vector2d(border_min_x,border_min_y)));
    Eigen::Vector2i max_point(jps_plan_.world2grid(Eigen::Vector2d(border_max_x,border_max_y)));
    int cross_num = 0;
    std::vector<Eigen::Vector2d> cross_vec;
    for(int j = point_grid(1);j>min_point(1) && j <= max_point(1);j = j + direction)
    {
        Eigen::Vector2i temp_point_grid(point_grid(0),j);
        Eigen::Vector2i next_point_grid(point_grid(0),j + direction);
        int temp_map_index = jps_plan_.coord2Id(temp_point_grid);
        int next_map_index = jps_plan_.coord2Id(next_point_grid);
        if(border_map.data[temp_map_index] != border_map.data[next_map_index])
        {
            if((border_map.data[next_map_index] == 80) && (border_map.data[temp_map_index] == 0))
            {
                Eigen::Vector2d temp_point = jps_plan_.grid2world(temp_point_grid);
                cross_vec.push_back(temp_point);
                cross_num ++;                  
            }
        }
    }
    // if(direction == 1)   ROS_INFO("search up,cross num = %d",cross_num);
    // if(direction == -1)  ROS_INFO("search down,cross num = %d",cross_num);
    // for(auto cross_point:cross_vec)     ROS_INFO("cross point x = %f,y=%f",cross_point(0),cross_point(1));
    if(cross_num % 2 == 0)     
    {
        return true;
    }
    return false;
}

//横向检查是否为沿边曲线外的点
bool COVER_SWEEP::isOutCruvePointByRow(const Eigen::Vector2i point_grid,const int direction)  
{
    Eigen::Vector2i min_point(jps_plan_.world2grid(Eigen::Vector2d(border_min_x,border_min_y)));
    Eigen::Vector2i max_point(jps_plan_.world2grid(Eigen::Vector2d(border_max_x,border_max_y)));
    int cross_num = 0;
    std::vector<Eigen::Vector2d> cross_vec;
    for(int j = point_grid(0);j>min_point(0) && j <= max_point(0);j = j + direction)
    {
        Eigen::Vector2i temp_point_grid(j,point_grid(1));
        Eigen::Vector2i next_point_grid(j + direction,point_grid(1));
        int temp_map_index = jps_plan_.coord2Id(temp_point_grid);
        int next_map_index = jps_plan_.coord2Id(next_point_grid);
        if(border_map.data[temp_map_index] != border_map.data[next_map_index])
        {
            if((border_map.data[next_map_index] == 80) && (border_map.data[temp_map_index] == 0))
            {
                Eigen::Vector2d temp_point = jps_plan_.grid2world(temp_point_grid);
                cross_vec.push_back(temp_point);
                cross_num ++;                  
            }
        }
    }
    // if(direction == 1)      ROS_INFO("search right,cross num = %d",cross_num);
    // if(direction == -1)     ROS_INFO("search left,cross num = %d",cross_num);
    // for(auto cross_point:cross_vec)     ROS_INFO("cross point x = %f,y=%f",cross_point(0),cross_point(1));
    if(cross_num % 2 == 0)     
    {
        return true;
    }
    return false;
}


float COVER_SWEEP::getJpsDistance(Eigen::Vector2d point1,Eigen::Vector2d point2)
{
    static int count = 0;
    ROS_INFO("use jps modle count = %d",++count);
    if(jps_plan_.globalPathPlaning(point1,point2,expand_map))
    {
        ROS_INFO("get jps path ok!");
        return jps_plan_.getPathLenght();
    }
    return 1000;
}

float COVER_SWEEP::getEuclideanDistance(Eigen::Vector2d point1,Eigen::Vector2d point2)
{
    float diff_x = point1(0) - point2(0);
    float diff_y = point1(1) - point2(1);
    return (sqrt(diff_x * diff_x + diff_y * diff_y));
}

bool COVER_SWEEP::isLineEmpty(std::vector<SweepPoint>& line_vec)        //一列中的visited全部为true，则返回true。
{
    for(int i = 0;i < line_vec.size();i++)
    {
        // if(line_vec.empty())       return false;
        if(!line_vec[i].visited)   return false;
    }
    return true;
}

bool COVER_SWEEP::collisionCheck(Eigen::Vector2d start,Eigen::Vector2d end,nav_msgs::OccupancyGrid& map,int occupied_value)
{
    Eigen::Vector2i start_grid = jps_plan_.world2grid(start);
    Eigen::Vector2i end_grid = jps_plan_.world2grid(end);
    int collision_count = 0;
    if(fabs(start_grid(0) - end_grid(0)) < 0.05)
    {
        int min_y = std::min(start_grid(1),end_grid(1));
        int max_y = std::max(start_grid(1),end_grid(1));
        for(int i = min_y;i < max_y;i ++)
        {
            Eigen::Vector2i temp(start_grid(0),i);
            // if(jps_plan_.isOccupied(temp,map,occupied_value))      collision_count++;
            // if(collision_count > 1)             return false; 
            if(jps_plan_.isOccupied(temp,map,occupied_value))       return false;
        }
    }
    else if(fabs(start_grid(1) - end_grid(1)) < 0.05)
    {
        int min_x = std::min(start_grid(0),end_grid(0));
        int max_x = std::max(start_grid(0),end_grid(0));
        for(int i = min_x;i < max_x;i ++)
        {
            Eigen::Vector2i temp(i,start_grid(1));
            // if(jps_plan_.isOccupied(temp,map,occupied_value))     collision_count++;
            // if(collision_count > 1)             return false;
             if(jps_plan_.isOccupied(temp,map,occupied_value))      return false;
        }
    }
    else
    {
        float discrete_res = 0.03;
        float distance = getEuclideanDistance(start,end);
        float sin_th = (end(1) - start(1)) / distance;
        float cos_th = (end(0) - start(0)) / distance;
        int discrete_num = (int)(distance / discrete_res);
        for(int i = 0; i< discrete_num;i++)
        {
            float x = start(0) + i*discrete_res * cos_th;
            float y = start(1) + i*discrete_res * sin_th;
            Eigen::Vector2d temp_point(x,y);
            Eigen::Vector2i temp_point_grid = jps_plan_.world2grid(temp_point);
            // if(jps_plan_.isOccupied(temp_point_grid,map,occupied_value))    collision_count++;
            // if(collision_count > 1)             return false;
            if(jps_plan_.isOccupied(temp_point_grid,map,occupied_value)) return false;
        }
    }
    return true;
}

//判断横向直线是否与未清扫的场边直线相交，若存在，则为无效的横向直线搜索
bool COVER_SWEEP::isVaildLine(Eigen::Vector2i current_index,Eigen::Vector2i& next_index)
{
    if(fabs(current_index(0) - next_index(0)) < 2)     return true;
    else
    {
        int min_x = std::min(current_index(0),next_index(0));
        int max_x = std::max(current_index(0),next_index(0));
        for(int i = min_x + 1;i < max_x;i++)
        {
            for(int j = 0;j < sweep_point_vec[i].size()-1;j++)
            {
                if(!sweep_point_vec[i][j].visited && !sweep_point_vec[i][j+1].visited)
                {
                    Eigen::Vector2d line1_start(sweep_point_vec[current_index(0)][current_index(1)].point);
                    Eigen::Vector2d line1_end(sweep_point_vec[next_index(0)][next_index(1)].point);
                    Eigen::Vector2d line2_start(sweep_point_vec[i][j].point);
                    Eigen::Vector2d line2_end(sweep_point_vec[i][j+1].point);
                    if(isLineCross(line1_start,line1_end,line2_start,line2_end))    return false;
                } 
            }
        }
        return true;
    }
}

//判断两条线段是否相交
bool COVER_SWEEP::isLineCross(Eigen::Vector2d line1_start,Eigen::Vector2d line1_end,Eigen::Vector2d line2_start,Eigen::Vector2d line2_end)
{
    float d1 = direction(line2_start,line2_end,line1_start);
    float d2 = direction(line2_start,line2_end,line1_end);
    float d3 = direction(line1_start,line1_end,line2_start);
    float d4 = direction(line1_start,line1_end,line2_end);
    if(d1*d2<0 && d3*d4<0)      return true;
    return false;
}




int main(int argc,char** argv)
{
    ros::init(argc, argv, "cover_sweep");
    COVER_SWEEP cs;
    ros::spin();
}