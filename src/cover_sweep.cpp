#include "cover_sweep/cover_sweep.h"

COVER_SWEEP::COVER_SWEEP():
            plan_finished(false),
            last_path_notLine(false)
{
    goal_sub_ = node_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, boost::bind(&COVER_SWEEP::goalCallBack, this, _1));
    map_sub_ = node_.subscribe<nav_msgs::OccupancyGrid>("/map", 1, boost::bind(&COVER_SWEEP::MapCallBack, this, _1));
    path_pub_ = node_.advertise<nav_msgs::Path>("sweep_path", 1, true);
    expandmap_pub_ = node_.advertise<nav_msgs::OccupancyGrid>("/expandmap",1,true);
    JPS_Search* jps_plan_ = new JPS_Search;
}

COVER_SWEEP::~COVER_SWEEP()
{
    
}

void COVER_SWEEP::MapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& _map)
{
    ROS_INFO("I have recived map");
    nav_msgs::OccupancyGrid map;
    /*map.header.stamp = ros::Time::now();
    map.header.frame_id = "map";
    map.info.width = _map->info.width;
    map.info.height = _map->info.height;
    map.info.origin.position.x = _map->info.origin.position.x;
    map.info.origin.position.y = _map->info.origin.position.y;
    map.info.resolution = _map->info.resolution;
    map.data.resize(map.info.width*map.info.height);
    map.data.insert(map.data.begin(),_map->data.begin(),_map->data.end());*/
    map = *_map;

    jps_plan_.expandMap(map,expand_map);
    jps_plan_.mapDataInit(expand_map);
    expandmap_pub_.publish(expand_map);
}

void COVER_SWEEP::goalCallBack(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
    ROS_INFO("I recived new goal!");
    Eigen::Vector2d point(goal->pose.position.x,goal->pose.position.y);
    zone_vec.push_back(point);
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";
    if(zone_vec.size() == 2)
    {
        plan_finished = false;
        coverPlan(zone_vec,path);
        zone_vec.clear();
        sweep_point_vec.clear();

    }
}

bool COVER_SWEEP::coverPlan(std::vector<Eigen::Vector2d>& zone,nav_msgs::Path& path)
{
    float min_x = std::min(zone[0](0),zone[1](0));
    float min_y = std::min(zone[0](1),zone[1](1));
    float max_x = std::max(zone[0](0),zone[1](0));
    float max_y = std::max(zone[0](1),zone[1](1));
    ROS_INFO("min x = %f,min y = %f,max x = %f,max y = %f",min_x,min_y,max_x,max_y);
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
    ros::Time start_time = ros::Time::now();
    full_cover(zone_vertx,path);
    ROS_INFO("cover plan cost time = %f",(ros::Time::now() - start_time).toSec());
    /*for(int k = 0;k < path.poses.size();k ++)
    {
        ROS_INFO("path node x =%f,y = %f",path.poses[k].pose.position.x,path.poses[k].pose.position.y);
    }*/
    path_pub_.publish(path);
}

bool COVER_SWEEP::full_cover(std::vector<Eigen::Vector2d>& zone_vertx,nav_msgs::Path& path)
{
    // Eigen::Vector2d start(zone_vertx[0](0) + SWEEP_GAP,zone_vertx[0](1) + SWEEP_GAP);
    int sweep_line_num = std::ceil((zone_vertx[1](0) - zone_vertx[0](0)) / SWEEP_GAP) - 1;
    ROS_INFO("sweep line num = %d",sweep_line_num);
    sweep_point_vec.resize(sweep_line_num);
    for(int i = 0 ;i < sweep_line_num; i ++)
    {
        find_line_point(zone_vertx,sweep_point_vec[i],i+1);
        for(int j = 0; j < sweep_point_vec[i].size();j ++)
           ROS_INFO("line %d,point index = %d",i,j);
    }
    ROS_INFO("find point ok");
    sweep_path_plan(path);
}

void COVER_SWEEP::find_line_point(std::vector<Eigen::Vector2d>& zone_vertx,std::vector<SweepPoint>& sweep_line_point,int index)
{
    /*static int index_y = 0;
    static int last_index_x = 0;
    if(last_index_x != index) {last_index_x = index;index_y = 0;}*/
    int num = 0;
    float line_x = zone_vertx[0](0) + index * SWEEP_GAP;
    Eigen::Vector2d start_point(line_x ,zone_vertx[0](1) + SWEEP_GAP);
    Eigen::Vector2d end_point(line_x ,zone_vertx[2](1) -  SWEEP_GAP);
    Eigen::Vector2i start_point_grid = jps_plan_.world2grid(start_point);
    Eigen::Vector2i end_point_grid = jps_plan_.world2grid(end_point);
    // ROS_INFO("start point in world x = %f,y =%f,end point = %f,y = %f",start_point(0),start_point(1),end_point(0),end_point(1));
    // ROS_INFO("start point in grid x = %d,y =%d,end point = %d,y = %d",start_point_grid(0),start_point_grid(1),end_point_grid(0),end_point_grid(1));
    int map_index_x = start_point_grid(0);
    int map_index_y_min = start_point_grid(1);
    int map_index_y_max = end_point_grid(1);
    /*如果起点为free，先往数组里插入头*/
    if(!jps_plan_.isOccupied(start_point_grid))    { SweepPoint start = {start_point,false};sweep_line_point.push_back(start);}
    /*遍历该条直线*/
    for(int i = map_index_y_min;i < map_index_y_max;i ++)
    {   
        // num++;
        // ROS_INFO("point index = %d",num);
        Eigen::Vector2i temp_point_grid(map_index_x,i);
        Eigen::Vector2i next_point_grid(map_index_x,i+1);
        if(!jps_plan_.isVaildCell(temp_point_grid))    {ROS_INFO("invalid point,");continue;}            //如果起始点是在地图外的无效点，则直接结束该行的搜索
        /*判断地图栅格信息是否发生变化*/
        int temp_map_index = jps_plan_.coord2Id(temp_point_grid);
        int next_map_index = jps_plan_.coord2Id(next_point_grid);
        if(expand_map.data[temp_map_index] != expand_map.data[next_map_index])
        {
            Eigen::Vector2d temp_point_world;
            if(expand_map.data[temp_map_index] == 100 && expand_map.data[next_map_index] == 0)
            {
                temp_point_world = jps_plan_.grid2world(Eigen::Vector2i(map_index_x,i+1));
                temp_point_world(0) =  line_x;
            }
            else if(expand_map.data[next_map_index] == 100 && expand_map.data[temp_map_index] == 0)
            {
                temp_point_world = jps_plan_.grid2world(Eigen::Vector2i(map_index_x,i));
                temp_point_world(0) =  line_x;
            }
            SweepPoint point = {temp_point_world,false};
            sweep_line_point.push_back(point);
        }
    }
    /*如果终点为free，最后再插入终点*/
    if(!jps_plan_.isOccupied(end_point_grid))    { SweepPoint end = {end_point,false};sweep_line_point.push_back(end);}
}


void COVER_SWEEP::sweep_path_plan(nav_msgs::Path& path)
{
    std::vector<Eigen::Vector2d> path_sweep;
    int init_index_x = 0;
    int init_index_y = 0;
    ROS_INFO("prepare start loop 01");
    path_sweep.push_back(sweep_point_vec[init_index_x][init_index_y].point);
    ROS_INFO("prepare start loop 011");
    sweep_point_vec[init_index_x][init_index_y].visited = true;
    current_point_index << init_index_x,init_index_y;
    while(!plan_finished)
    {
        Eigen::Vector2d current_point = path_sweep.back();
        find_next_point(current_point_index,path_sweep);
    }
    for(int i = 0 ;i < path_sweep.size();i++)
    {
        geometry_msgs::PoseStamped temp;
        Eigen::Vector2d temp_point = jps_plan_.grid2world(jps_plan_.world2grid(path_sweep[i]));
        temp.pose.position.x = temp_point(0)/*path_sweep[i](0)*/;
        temp.pose.position.y = temp_point(1)/*path_sweep[i](1)*/;
        path.poses.push_back(temp);
    }
}

float COVER_SWEEP::getChebyshevDistance(Eigen::Vector2d point1,Eigen::Vector2d point2)
{
    return std::max(fabs(point1(0) - point2(0)),fabs(point1(1) - point2(1)));
}

float COVER_SWEEP::getManhattanDistance(Eigen::Vector2d point1,Eigen::Vector2d point2)
{
    return (fabs(point1(0) - point2(0)) + fabs(point1(1) - point2(1)));
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
    return 100;
}

float COVER_SWEEP::getEuclideanDistance(Eigen::Vector2d point1,Eigen::Vector2d point2)
{
    float diff_x = point1(0) - point2(0);
    float diff_y = point1(1) - point2(1);
    return (sqrt(diff_x * diff_x + diff_y * diff_y));
}

bool COVER_SWEEP::isLineEmpty(std::vector<SweepPoint>& line_vec)
{
    for(int i = 0;i < line_vec.size();i++)
    {
        if(line_vec.empty())       return false;
        if(!line_vec[i].visited)   return false;
    }
    return true;
}

bool COVER_SWEEP::isNearLineEmpty(Eigen::Vector2i& current_index)
{   //ROS_INFO("run to here 3.1.1");
    int max_line_num = sweep_point_vec.size();
    int min_x = std::max(0,current_index(0) - 3);
    int max_x = std::min((current_index(0) + 4),max_line_num);
    //ROS_INFO("run to here 3.1.2");
    for(int i = min_x;i < max_x;i++)
    {
        //ROS_INFO("check line %d is empty",i);
        if(isLineEmpty(sweep_point_vec[i]))    return false;
    }
    //ROS_INFO("run to here 3.1.3");
    return true;
}

void COVER_SWEEP::find_next_point(Eigen::Vector2i& current_index,std::vector<Eigen::Vector2d>& path_sweep)
{
    ROS_INFO("!!!!!!!!!---------------current index x = %d,y = %d,prepare find next point-----------------------!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!",current_index(0),current_index(1));
    float max_distance_up = 100,max_distance_down = 100,min_distance_next = 100,temp_min_distance = 100;
    Eigen::Vector2i next_index_up,next_index_down;
    std::vector<Eigen::Vector2d> next_point_up_vec,next_point_down_vec,next_line_point_vec,temp_next_point_vec;
    bool jump_up_status = jump(current_index,next_index_up,next_point_up_vec,1,max_distance_up);        //向上搜索下一个点
    //ROS_INFO("jump up status = %d",jump_up_status);
    bool jump_down_status = jump(current_index,next_index_down,next_point_down_vec,-1,max_distance_down); //向下搜索下一个点
    //ROS_INFO("jump down status = %d",jump_down_status);
    if(!jump_up_status && !jump_down_status)
    {
        last_point_index = current_index;
        /*向上搜索和向下搜索都失败，则该列搜索暂时结束，搜索前一列，前一列搜索成功则走前一列，反之则搜索下一列*/
        if(find_next_line_start(current_index,next_line_point_vec,min_distance_next))  
        {
            path_sweep.insert(path_sweep.end(),next_line_point_vec.begin(),next_line_point_vec.end());
            //current_index更改其访问状态
            sweep_point_vec[current_index(0)][current_index(1)].visited = true;
            ROS_INFO("next line ok,new node x = %d,y = %d",current_index(0),current_index(1));
        }
        else
        {
            plan_finished = true;
            ROS_INFO("finish search!");
        }
    }
    else if(jump_up_status && !jump_down_status)
    {
        /*向上搜索成功，向下搜索失败，则选用向上的点作为next_point，并更新current_index*/
        path_sweep.insert(path_sweep.end(),next_point_up_vec.begin(),next_point_up_vec.end());
        last_point_index = current_index;
        current_index = next_index_up;
        sweep_point_vec[current_index(0)][current_index(1)].visited = true;
        ROS_INFO("search up ok,new node x = %d,y = %d",current_index(0),current_index(1));
    }
    else if(!jump_up_status && jump_down_status)
    {
        /*向下搜索成功，向上搜索失败，则选用向下的点作为next_point，并更新current_index*/
        path_sweep.insert(path_sweep.end(),next_point_down_vec.begin(),next_point_down_vec.end());
        last_point_index = current_index;
        current_index << next_index_down(0),next_index_down(1);
        sweep_point_vec[current_index(0)][current_index(1)].visited = true;
        ROS_INFO("search down ok,new node x = %d,y = %d",current_index(0),current_index(1));
    }
    else if(jump_up_status && jump_down_status)
    {
        /*向上搜索和向下搜索都成功，则选用max_distance小的作为next_point，并更新current_index*/
        if(max_distance_down < max_distance_up)     
        {
            path_sweep.insert(path_sweep.end(),next_point_down_vec.begin(),next_point_down_vec.end());
            last_point_index = current_index;
            current_index = next_index_down;
            sweep_point_vec[current_index(0)][current_index(1)].visited = true;
            ROS_INFO("search down ok,new node x = %d,y = %d",current_index(0),current_index(1));
        }
        else                                        
        {   
            path_sweep.insert(path_sweep.end(),next_point_up_vec.begin(),next_point_up_vec.end());
            last_point_index = current_index;
            current_index = next_index_up;
            sweep_point_vec[current_index(0)][current_index(1)].visited = true;
            ROS_INFO("search up ok,new node x = %d,y = %d",current_index(0),current_index(1));
        }
    }
}

bool COVER_SWEEP::find_next_line_start(Eigen::Vector2i& current_index,std::vector<Eigen::Vector2d>& next_point_vec,float& distance)
{
    /*首先向左搜索，向左搜索失败则向右搜索*/
    if(jump2NextLine(current_index,next_point_vec,-1,distance))      //首先向左搜索
    {
        // ROS_INFO("find left ok!");
        return true;
    }
    else if (jump2NextLine(current_index,next_point_vec,1,distance))     //其次向右搜索
    {
        // ROS_INFO("find left fail,find right ok!");
        return true;
    }
    else                                                    //若左右搜索都失败，则路径规划结束
    {
        ROS_INFO("find next line fail!");
        return false;
    }
}

bool COVER_SWEEP::getLeftLinePoint_ByStraight(Eigen::Vector2i& current_index,Eigen::Vector2i& next_index,std::vector<Eigen::Vector2d>& next_point_vec)
{
    int current_x = current_index(0);
    int current_y = current_index(1);
    // for(int m = current_x;m  >=0;m--)
    for(int m = 1;m < current_x;m++)
    {
        if(m - 1 <0)     return false;
        if(!isLineEmpty(sweep_point_vec[m - 1]))
        {
            float min_distance = 100;
            for(int i = 0 ;i < sweep_point_vec[m - 1].size();i++)
            {
                if(sweep_point_vec[m -1][i].visited)  continue;
                Eigen::Vector2d current_point(sweep_point_vec[current_x][current_y].point);
                Eigen::Vector2d temp_point(sweep_point_vec[m -1][i].point);
                if(collisionCheck(current_point,temp_point))
                {
                    float euclidean_dis = getEuclideanDistance(current_point,temp_point);
                    if(euclidean_dis < min_distance)
                    {
                        next_point_vec.clear();
                        next_point_vec.push_back(temp_point);
                        min_distance = euclidean_dis;
                        next_index << m - 1,i;
                    }
                }
            } 
            if(min_distance == 100)   continue;
            else                      return true;
        }
    }
    return false;
}


bool COVER_SWEEP::getRightLinePoint_ByStraight(Eigen::Vector2i& current_index,Eigen::Vector2i& next_index,std::vector<Eigen::Vector2d>& next_point_vec)
{
    int current_x = current_index(0);
    int current_y = current_index(1);
    for(int m = current_x;m < sweep_point_vec.size();m++)
    {
        if(m+1 >=sweep_point_vec.size())     return false;
        if(!isLineEmpty(sweep_point_vec[m+1]))
        {
            float min_distance = 100;
            for(int i = 0 ;i < sweep_point_vec[m+1].size();i++)
            {
                if(sweep_point_vec[m +1][i].visited)  continue;
                Eigen::Vector2d current_point(sweep_point_vec[current_x][current_y].point);
                Eigen::Vector2d temp_point(sweep_point_vec[m +1][i].point);
                if(collisionCheck(current_point,temp_point))
                {
                    float euclidean_dis = getEuclideanDistance(current_point,temp_point);
                    if(euclidean_dis < min_distance)
                    {
                        next_point_vec.clear();
                        next_point_vec.push_back(temp_point);
                        min_distance = euclidean_dis;
                        next_index << m+1,i;
                    }
                }
            } 
            if(min_distance == 100)   continue;
            else                      return true;
        }
    }
    return false;
}

bool COVER_SWEEP::jump2NextLine(Eigen::Vector2i& current_index,std::vector<Eigen::Vector2d>& next_point_vec,int direction/*1:向右搜索  -1：向左搜索*/,float& distance)
{
    int current_x = current_index(0);
    int current_y = current_index(1);
    for(int m = current_x;(m  >=0 && m < sweep_point_vec.size());m = m + direction)
    {
        // ROS_INFO("search m = %d,direction = %d",m,direction);
        if(direction == -1 && m -1 <0)    return false;
        if(direction == 1 && m +1 >= sweep_point_vec.size())    continue;
        if(!isLineEmpty(sweep_point_vec[m + direction]))
        {
            float min_distance = 100;
            bool isLinePass = false,isManhattanPass = false;
            for(int i = 0 ;i < sweep_point_vec[m + direction].size();i++)
            {
                ROS_INFO("find next point x = %d,y = %d",m + direction,i);
                if(sweep_point_vec[m + direction][i].visited)  continue;
                Eigen::Vector2d current_point(sweep_point_vec[current_x][current_y].point);
                Eigen::Vector2d temp_point(sweep_point_vec[m + direction][i].point);
                std::vector<Eigen::Vector2d> temp_point_vec;  
                float euclidean_dis,manhattan_dis,jps_distance;
                euclidean_dis = getEuclideanDistance(current_point,temp_point);
                manhattan_dis = getManhattanDistance(current_point,temp_point);        

                if(collisionCheck(current_point,temp_point) && euclidean_dis < min_distance)
                {
                    ROS_INFO("-----------lien to next point ok,next index = %d,y = %d,distance = %f",m + direction,i,euclidean_dis);
                    next_point_vec.clear();
                    next_point_vec.push_back(temp_point);
                    min_distance = euclidean_dis;
                    current_index << m + direction,i;
                    isLinePass = true;
                }
                /*else if(manhattan_dis <= min_distance && getManhattanPath(current_point,temp_point,temp_point_vec))
                {       
                    ROS_INFO("**********manhattan to next point ok,next index = %d,y = %d,distance = %f",m + direction,i,manhattan_dis);            
                    next_point_vec.clear();
                    next_point_vec.insert(next_point_vec.begin(),temp_point_vec.begin(),temp_point_vec.end());
                    min_distance = manhattan_dis;
                    current_index << m + direction,i;
                    isManhattanPass = true;
                }*/
                else if(!isLinePass && !isManhattanPass)
                {
                    // Eigen::Vector2i next_index_right,next_index_left;
                    // bool left_status = getLeftLinePoint_ByStraight(current_index,next_index_left,next_point_vec);
                    // bool right_status = getRightLinePoint_ByStraight(current_index,next_index_right,next_point_vec);
                    // if(direction == 1 && left_status)   {current_index = next_index_left;return true;}
                    // else if(direction == -1 && right_status)    {current_index = next_index_right;return true;}
                    // if(direction == -1 && getRightLinePoint_ByStraight(current_index,next_point_vec))    return true;
                    // if(direction ==  1 && getLeftLinePoint_ByStraight(current_index,next_point_vec))     return true;
                    jps_distance = getJpsDistance(current_point,temp_point);
                    // float chebyshev_dis = getChebyshevDistance(current_point,temp_point);
                    ROS_INFO("++++++++++jps next index :x = %d,y = %d,distance = %f",m + direction,i,jps_distance);
                    if((jps_distance != 100 && jps_distance < min_distance)/* && (manhattan_dis - jps_distance <= SWEEP_GAP)*/)
                    {
                        nav_msgs::Path temp_path = jps_plan_.getGlobalPath();
                        getJpsPath(temp_path,temp_point_vec);   
                        temp_point_vec[0](0) = current_point(0);
                        temp_point_vec[0](1) = current_point(1);
                        temp_point_vec[temp_point_vec.size()-1](0) = temp_point(0);
                        temp_point_vec[temp_point_vec.size()-1](1) = temp_point(1); 
                        next_point_vec.clear();
                        next_point_vec.insert(next_point_vec.begin(),temp_point_vec.begin(),temp_point_vec.end());
                        min_distance = jps_distance;
                        current_index << m + direction,i;
                    }
                }
            }
            if(min_distance == 100)    continue;
            
            distance = min_distance;
            return true;
        }
    }

    return false;
} 


bool COVER_SWEEP::jump(Eigen::Vector2i& current_index,Eigen::Vector2i& next_index,std::vector<Eigen::Vector2d>& next_point_vec,int direction/*1:向上搜索   -1：向下搜索*/,float& max_distance)
{
    int x= current_index(0);
    int y= current_index(1);
    int max_y = sweep_point_vec[x].size();          //y方向最大的点的序号
    int min_y = 0;
    bool isGetNextPoint = false;             //是否已经获得了下一个目标点
    next_point_vec.clear();
    if(direction == 1)                  //向上搜索
    {
        for(int i = y + 1; i < max_y; i++)
        {
             //ROS_INFO("search up,index x = %d,y = %d!",x,i);
            if(sweep_point_vec[x][i].visited)  break;
            Eigen::Vector2d current_point;
            if(!isGetNextPoint)
            {
                current_point = sweep_point_vec[x][y].point;
            }
            else
            {
                current_point = sweep_point_vec[x][i-1].point;
            }
            //ROS_INFO("run to here 0");
            Eigen::Vector2d temp_point(sweep_point_vec[x][i].point);
            float euclidean_dis = getEuclideanDistance(current_point,temp_point);
            float line3rectangle_dis = 0;
            std::vector<Eigen::Vector2d> temp_path_vec;
            //ROS_INFO("run to here 1");
            /*if(last_path_notLine)
            {
                if(getLeftLinePoint_ByStraight(current_index,next_index,temp_path_vec))
                {
                    next_point_vec = temp_path_vec;
                    isGetNextPoint = true;
                    last_path_notLine = false; 
                    sweep_point_vec[current_index(0)][current_index(1)].visited = false;
                    goto jumpsuccess;
                }

            }*/
            //ROS_INFO("run to here 1");
            if(collisionCheck(current_point,temp_point))  
            {
                if(!isGetNextPoint) 
                {
                    ROS_INFO("line up pass!");
                    next_point_vec.push_back(temp_point);
                    isGetNextPoint = true;
                    last_path_notLine = false;
                    next_index << current_index(0),i;
                }
                max_distance += euclidean_dis;
                // continue;
                goto jumpsuccess;
            }
            //ROS_INFO("run to here 2");
            /*if(get3LineRectangle(current_point,temp_point,temp_path_vec,line3rectangle_dis))
            {
                if(!isGetNextPoint) 
                {
                    // ROS_INFO("start x = %f,y = %f,end x= %f,y = %f",current_point(0),current_point(1),temp_point(0),temp_point(1));
                    for(int k = 0;k < next_point_vec.size();k++)
                    {
                        ROS_INFO("line 3 point %d,x = %f,y =%f",k+1,next_point_vec[k](0),next_point_vec[k](1));
                    }
                    // ROS_INFO("3 line rectangle up pass!,vec size = %d",next_point_vec.size());
                    next_point_vec = temp_path_vec;
                    isGetNextPoint = true;
                    last_path_notLine = true;
                    next_index << current_index(0),i;
                }
                max_distance += line3rectangle_dis;
                // continue;
                goto jumpsuccess;
            }*/
            //ROS_INFO("run to here 3");
            /*if(!isNearLineEmpty(current_index))
            {
                //ROS_INFO("run to here 3.1");
                float jps_distance = getJpsDistance(current_point,temp_point);
                // ROS_INFO("search up,from %d to %d,jps distance = %f,euclidean = %f",current_index(1),current_index(1)+1,jps_distance,euclidean_dis);
                //ROS_INFO("run to here 3.2");
                if(jps_distance != 100 && jps_distance - euclidean_dis <= 1.0 *SWEEP_GAP)
                {   //ROS_INFO("run to here 3.3");
                    if(!isGetNextPoint) 
                    {
                        ROS_INFO("jps up pass!");
                        nav_msgs::Path temp_path = jps_plan_.getGlobalPath();
                        getJpsPath(temp_path,next_point_vec);
                        next_point_vec[0](0) = current_point(0);
                        next_point_vec[0](1) = current_point(1);
                        next_point_vec[next_point_vec.size()-1](0) = temp_point(0);
                        next_point_vec[next_point_vec.size()-1](1) = temp_point(1);
                        isGetNextPoint = true;
                        last_path_notLine = true;
                        next_index << current_index(0),i;
                        goto jumpsuccess;
                    }
                    max_distance += jps_distance;
                }
                else
                {
                    break;
                }
            }*/
            //ROS_INFO("run to here 4");
        }
    }
    else if(direction == -1)             //向下搜索
    {
        for(int i = y -1; i >= min_y;i -- )
        {
            if(sweep_point_vec[x][i].visited)  break;
            Eigen::Vector2d current_point;
            if(!isGetNextPoint)
            {
                current_point = sweep_point_vec[x][y].point;
            }
            else
            {
                current_point = sweep_point_vec[x][i-1].point;
            }
            Eigen::Vector2d temp_point(sweep_point_vec[x][i].point);
            float euclidean_dis = getEuclideanDistance(current_point,temp_point);
            float line3rectangle_dis = 0;
            std::vector<Eigen::Vector2d> temp_path_vec;
            /*if(last_path_notLine && getLeftLinePoint_ByStraight(current_index,next_index,temp_path_vec))
            {
                next_point_vec = temp_path_vec;
                isGetNextPoint = true;
                last_path_notLine = false; 
                sweep_point_vec[current_index(0)][current_index(1)].visited = false;
                goto jumpsuccess;
            }*/
            if(collisionCheck(current_point,temp_point))  
            {
                // ROS_INFO("line %d,point %d to %d is passed",current_index(0),y,i);
                
                if(!isGetNextPoint) 
                {
                    ROS_INFO("line down pass!");
                    next_point_vec.push_back(temp_point);
                    isGetNextPoint = true;
                    last_path_notLine = false;
                    next_index << current_index(0),i;
                }
                max_distance += euclidean_dis;
                // continue;
                goto jumpsuccess;
            }
            /*else if(get3LineRectangle(current_point,temp_point,temp_path_vec,line3rectangle_dis))
            { 
                if(!isGetNextPoint) 
                {
                    for(int k = 0;k < next_point_vec.size();k++)
                    {
                        ROS_INFO("line 3 point %d,x = %f,y =%f",k+1,next_point_vec[k](0),next_point_vec[k](1));
                    }
                    isGetNextPoint = true;
                    last_path_notLine = true;
                    next_index << current_index(0),i;
                    next_point_vec = temp_path_vec;
                }
                max_distance += line3rectangle_dis;
                // continue;
                goto jumpsuccess;
            }*/
            /*else if(!isNearLineEmpty(current_index))
            {
                float jps_distance = getJpsDistance(current_point,temp_point);    //暂时用曼哈顿距离代替路径规划距离
                // ROS_INFO("search down,from %d to %d,jps distance = %f,euclidean = %f",current_index(1),current_index(1)-1,jps_distance,euclidean_dis);
                ROS_INFO("use jps find path");
                if(jps_distance != 100 && jps_distance - euclidean_dis <= 1.0 *SWEEP_GAP)
                {
                    if(!isGetNextPoint) 
                    {
                        ROS_INFO("jps up pass!");
                        nav_msgs::Path temp_path = jps_plan_.getGlobalPath();
                        getJpsPath(temp_path,next_point_vec);
                        next_point_vec[0](0) = current_point(0);
                        next_point_vec[0](1) = current_point(1);
                        next_point_vec[next_point_vec.size()-1](0) = temp_point(0);
                        next_point_vec[next_point_vec.size()-1](1) = temp_point(1);
                        isGetNextPoint = true;
                        last_path_notLine = true;
                        next_index << current_index(0),i;
                        goto jumpsuccess;
                    }
                    max_distance += jps_distance;
                }
                else
                {
                    break;
                }
            }*/
        }
    }
    if(!isGetNextPoint)     return false;

jumpsuccess:
    isGetNextPoint = false;
    if(current_index(0) != last_point_index(0) && (current_index(1) / 2) != (next_index(1) /2))
        {
            sweep_point_vec[current_index(0)][current_index(1)].visited = false;
            ROS_INFO("invalid start index : x = %d,y = %d",current_index(0),current_index(1));
        }
    return true;
}

bool COVER_SWEEP::collisionCheck(Eigen::Vector2d start,Eigen::Vector2d end)
{
    Eigen::Vector2i start_grid = jps_plan_.world2grid(start);
    Eigen::Vector2i end_grid = jps_plan_.world2grid(end);
    if(fabs(start_grid(0) - end_grid(0)) < 0.05)
    {
        int min_y = std::min(start_grid(1),end_grid(1));
        int max_y = std::max(start_grid(1),end_grid(1));
        for(int i = min_y;i < max_y;i ++)
        {
            Eigen::Vector2i temp(start_grid(0),i);
            if(jps_plan_.isOccupied(temp))     return false;
        }
    }
    else if(fabs(start_grid(1) - end_grid(1)) < 0.05)
    {
        int min_x = std::min(start_grid(0),end_grid(0));
        int max_x = std::max(start_grid(0),end_grid(0));
        for(int i = min_x;i < max_x;i ++)
        {
            Eigen::Vector2i temp(i,start_grid(1));
            if(jps_plan_.isOccupied(temp))     return false;
        }
    }
    else
    {
        float discrete_res = 0.1;
        float distance = getEuclideanDistance(start,end);
        float sin_th = (end(1) - start(1)) / distance;
        float cos_th = (end(0) - start(0)) / distance;
        int discrete_num = (int)(distance / discrete_res);
        for(int i = 0; i< discrete_num;i++)
        {
            float x = start(0) + i*discrete_res * cos_th;
            float y = start(1) + i*discrete_res * sin_th;
            Eigen::Vector2d temp_point(x,y);
            if(jps_plan_.isOccupied(jps_plan_.world2grid(temp_point)))    return false;
        }
    }
    return true;
}

void COVER_SWEEP::getJpsPath(nav_msgs::Path temp_path,std::vector<Eigen::Vector2d>& path_vec)
{
    for(int i = 0; i < temp_path.poses.size();i ++)
    {
        Eigen::Vector2d temp_point;
        temp_point<< temp_path.poses[i].pose.position.x,temp_path.poses[i].pose.position.y;
        path_vec.push_back(temp_point);
    }
}

bool COVER_SWEEP::getManhattanPath(Eigen::Vector2d start,Eigen::Vector2d end,std::vector<Eigen::Vector2d>& manhattan_path)
{
    Eigen::Vector2d temp_point1(start(0),end(1));
    Eigen::Vector2d temp_point2(end(0),start(1));
    if(collisionCheck(start,temp_point1) && collisionCheck(temp_point1,end))
    {
        manhattan_path.push_back(start);
        manhattan_path.push_back(temp_point1);
        manhattan_path.push_back(end);
        return true;
    }
    else if(collisionCheck(start,temp_point2) && collisionCheck(temp_point2,end))
    {
        manhattan_path.push_back(start);
        manhattan_path.push_back(temp_point2);
        manhattan_path.push_back(end);
        return true;        
    }
    return false;
}

/*
line3rectangle:        -----------                  -------------------
                       |                                              |
                       |                 or                           |
                       |                                              |
                       -----------                   ------------------
*/
bool COVER_SWEEP::get3LineRectangle(Eigen::Vector2d start,Eigen::Vector2d end,std::vector<Eigen::Vector2d>& line3rectangle_path,float& distance)
{
    // ROS_INFO("use 3line rectangle");
    line3rectangle_path.clear();
    bool last_line_status = false;
    for(int i = -4;i < 5;i++)
    {
        bool current_line_status = false;
        // ROS_INFO("i = %d",i);
        if(i == 0)    continue;
        Eigen::Vector2d temp_point1(start(0) + (float)(i * MAP_RESELUTION),start(1));
        Eigen::Vector2d temp_point2(start(0) + (float)(i * MAP_RESELUTION),end(1));
        if(collisionCheck(start,temp_point1) && collisionCheck(temp_point1,temp_point2) && collisionCheck(temp_point2,end))
        {
            line3rectangle_path.clear();
            line3rectangle_path.push_back(start);
            line3rectangle_path.push_back(temp_point1);
            line3rectangle_path.push_back(temp_point2);
            line3rectangle_path.push_back(end);
            distance = getEuclideanDistance(start,temp_point1) + getEuclideanDistance(temp_point1,temp_point2) + getEuclideanDistance(temp_point2,end);
            current_line_status = true;
        }
        if(i < 0 && (!current_line_status && last_line_status))   break;
        if(i > 0 && (current_line_status && !last_line_status))   break;
        last_line_status = current_line_status;
    }
    if(line3rectangle_path.empty())       return false;  
    return true;
}



int main(int argc,char** argv)
{
    ros::init(argc, argv, "cover_sweep");
    COVER_SWEEP cs;
    ros::spin();
}