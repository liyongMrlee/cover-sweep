#include "cover_sweep/jps_search.h"
#include <ros/ros.h>
#include <math.h>

using namespace std;
using namespace Eigen;

bool isPathPlaning = false;

JPS_Search::JPS_Search():
            map_size(0),
            map_width(0),
            map_height(0),
            map_origin_x(0),
            map_origin_y(0),
            map_resolution(0),
            path_length(0),
            invalid_start(false),
            invalid_goal(false)
{

}

JPS_Search::~JPS_Search()
{
   //ROS_INFO("delete jps search!");
}


void JPS_Search::expandMap(nav_msgs::OccupancyGrid map,nav_msgs::OccupancyGrid& expand_map,int occupied_value)
{
    int map_size = map.info.width * map.info.height;
    std::vector<int> map_data;
    map_data.resize(map_size);
    float map_resolution = map.info.resolution;
    int map_width = map.info.width;
    int map_height = map.info.height;
    // for(int i = 0;i<map_size;i++)   map_data[i] = (int)map.data[i];
    map_data.insert(map_data.begin(),map.data.begin(),map.data.end());
    double distance = ROBOT_RADIUS + SAFE_DISTANCE;          //路径节点与障碍物应当保持的距离
    int cell_num = std::ceil(distance / map_resolution);     //安全距离的栅格数
    //ROS_INFO("expand cell num = %d",cell_num);
    for(int x = 0;x < map_width;x++)
    {
      for(int y = 0;y< map_height;y++)
      {
        if(map.data[y * map_width + x] == occupied_value)
        // if(map.data[y * map_width + x] != -1 && map.data[y * map_width + x] != 0)
        expandMapCell(cell_num,Eigen::Vector2i(x,y),map_data,map,occupied_value);
      }
    }
     expand_map.header.stamp = ros::Time::now();
     expand_map.header.frame_id = "map";
     expand_map.info.width = map_width;
     expand_map.info.height = map_height;
     expand_map.info.origin.position.x = map.info.origin.position.x;
     expand_map.info.origin.position.y = map.info.origin.position.y;
     expand_map.info.resolution = map_resolution;
     expand_map.data.resize(map_size);
     /*for(int index = 0;i<map_size;i++)
     {
      expand_map.data[i] = map_data[i];
     }*/
     expand_map.data.insert(expand_map.data.begin(),map_data.begin(),map_data.end());
} 


void JPS_Search::expandMapCell(int cell_num,Eigen::Vector2i cell,std::vector<int>& map_data,nav_msgs::OccupancyGrid& map,int occupied_value)
{
  int cell_num_x_min = 0,cell_num_x_max=0,cell_num_y_min=0,cell_num_y_max=0;
  Eigen::Vector2i cell_temp;
  //先判断左边，对cell_num_x_min进行赋值
  cell_temp << cell(0)-1,cell(1);
  if(isValidCell(cell_temp,map) &&!isOccupied(cell_temp,map,occupied_value))    cell_num_x_min = -cell_num;
  //其次判断右边，对cell_num_x_max进行赋值
  cell_temp << cell(0)+1,cell(1);
  if(isValidCell(cell_temp,map) &&!isOccupied(cell_temp,map,occupied_value))    cell_num_x_max = cell_num;
  //再次判断下边，对cell_num_y_min进行赋值
  cell_temp << cell(0),cell(1)-1;
  if(isValidCell(cell_temp,map) &&!isOccupied(cell_temp,map,occupied_value))    cell_num_y_min = -cell_num;
  //最后判断上边，对cell_num_y_max进行赋值
  cell_temp << cell(0),cell(1)+1;
  if(isValidCell(cell_temp,map) &&!isOccupied(cell_temp,map,occupied_value))    cell_num_y_max = cell_num;

  for(int x = cell_num_x_min;x<= cell_num_x_max;x++)
  {
    for(int y = cell_num_y_min;y<= cell_num_y_max;y++)
    {
        int cell_x = cell(0) + x;
        int cell_y = cell(1) + y;
        cell_temp<< cell_x,cell_y;
        if(!isValidCell(cell_temp,map))    continue;
        map_data[cell_y * map.info.width + cell_x] = occupied_value;
    }
  }
} 

bool JPS_Search::isOccupied(Eigen::Vector2i& cell,nav_msgs::OccupancyGrid& map,int occupied_value)
{
  if(map.data[cell(1)*map.info.width + cell(0)] != occupied_value)   return false;
  return true;
}


bool JPS_Search::isValidCell(Eigen::Vector2i& cell,nav_msgs::OccupancyGrid& map)
{
    if ((cell(0) < 0) || (cell(0) >= map.info.width) || 
        (cell(1) < 0) || (cell(1) >= map.info.height))   return false;
    return true;
}

bool JPS_Search::collisionCheck(Eigen::Vector2d start,Eigen::Vector2d end,nav_msgs::OccupancyGrid& map)
{
    Eigen::Vector2i start_grid = world2grid(start);
    Eigen::Vector2i end_grid = world2grid(end);
    if(fabs(start_grid(0) - end_grid(0)) < 0.05)
    {
        int min_y = std::min(start_grid(1),end_grid(1));
        int max_y = std::max(start_grid(1),end_grid(1));
        for(int i = min_y;i < max_y;i ++)
        {
            Eigen::Vector2i temp(start_grid(0),i);
            if(isOccupied(temp,map,100))     return false;
        }
    }
    else if(fabs(start_grid(1) - end_grid(1)) < 0.05)
    {
        int min_x = std::min(start_grid(0),end_grid(0));
        int max_x = std::max(start_grid(0),end_grid(0));
        for(int i = min_x;i < max_x;i ++)
        {
            Eigen::Vector2i temp(i,start_grid(1));
            if(isOccupied(temp,map,100))     return false;
        }
    }
    else
    {
        float discrete_res = 0.1;
        // float distance = getEuclideanDistance(start,end);
        float distance = sqrt(pow(start(0) - start(0),2) + pow(end(1) - end(1),2));
        float sin_th = (end(1) - start(1)) / distance;
        float cos_th = (end(0) - start(0)) / distance;
        int discrete_num = (int)(distance / discrete_res);
        for(int i = 0; i< discrete_num;i++)
        {
            float x = start(0) + i*discrete_res * cos_th;
            float y = start(1) + i*discrete_res * sin_th;
            Eigen::Vector2d temp_point(x,y);
            Eigen::Vector2i temp_point_grid = world2grid(temp_point);
            if(isOccupied(temp_point_grid,map,100))    return false;
        }
    }
    return true;
}


void JPS_Search::mapDataInit(nav_msgs::OccupancyGrid& map)
{
     map_width = map.info.width;
     map_height = map.info.height;
     map_origin_x = map.info.origin.position.x;
     map_origin_y = map.info.origin.position.y;
     map_resolution = map.info.resolution;
     map_size = map_width * map_height;
    //  ROS_INFO("############map width = %d,map height = %d,origin_x = %f,origin_y = %f,res = %f",map_width,map_height,map_origin_x,map_origin_y,map_resolution);
     map_data.clear();
     map_data.resize(map_size);
     map_data.insert(map_data.begin(),map.data.begin(),map.data.end());
}

 void JPS_Search::JPS_init(Eigen::Vector2d _current_pose,Eigen::Vector2d _goal_pose,nav_msgs::OccupancyGrid& map)
 {
     start_pose << _current_pose(0),_current_pose(1);
     goal_pose <<  _goal_pose(0), _goal_pose(1);
     mapDataInit(map);
     jn2d_ = std::make_shared<JPS2DNeib>();
     pq_.clear();
     path_.clear();
     global_path.poses.clear();
     hm_.clear();
     hm_.resize(map_width * map_height);
     seen_.clear();
     seen_.resize(map_width * map_height, false); 
 }

  bool JPS_Search::tryStaightPath(Eigen::Vector2d _current_pose,Eigen::Vector2d _goal_pose,nav_msgs::OccupancyGrid& map)
  {
    if(collisionCheck(_current_pose,_goal_pose,map))
    {
      ROS_INFO("jps line pass");
        path_length = sqrt(pow(_current_pose(0) - _goal_pose(0),2) + pow(_current_pose(1) - _goal_pose(1),2));
        geometry_msgs::PoseStamped start_node,goal_node;
        start_node.pose.position.x = _current_pose(0);
        start_node.pose.position.y = _current_pose(1);
        goal_node.pose.position.x = _goal_pose(0);
        goal_node.pose.position.y = _goal_pose(1);
        global_path.poses.push_back(start_node);
        global_path.poses.push_back(goal_node);
        return true;
    }
    return false;
  }

 bool JPS_Search::globalPathPlaning(Eigen::Vector2d _current_pose,Eigen::Vector2d _goal_pose,nav_msgs::OccupancyGrid& map)
 {
    // if(tryStaightPath(_current_pose,_goal_pose,map))      return true;
    ROS_INFO("start x = %f,y = %f,Goal x= %f,y= %f",_current_pose(0),_current_pose(1),_goal_pose(0),_goal_pose(1));
    JPS_init(_current_pose,_goal_pose,map);  //JPS路径搜索初始化
    if(JPS_path_search(start_pose,goal_pose))  {return true;}
    return false; 
 }

 bool JPS_Search::findNearPoint(Eigen::Vector2i& point,int flag,int expand_num)
 {
    double distance = ROBOT_RADIUS + expand_num*SAFE_DISTANCE;          //路径节点与障碍物应当保持的距离
    int cell_num = std::ceil(distance / map_resolution);     //安全距离的栅格数 
    //std::cout<<"cell num = "<<cell_num<<std::endl;
    float min_distance = 10000.0;
    Eigen::Vector2i invalid_point(point);
    for(int x = -cell_num;x < cell_num;x++)
    {
      for(int y = -cell_num;y < cell_num;y++)
      {
         int cell_x = invalid_point(0) + x;
         int cell_y = invalid_point(1) + y;
         if(!isVaildCell(Eigen::Vector2i(cell_x,cell_y)))    continue;
         /*if(flag == 1 && plan_type == 1)
         {
           float zone_x = (float)current_zone.rect[0].y / 1000.0;
           float zone_y = -(float)current_zone.rect[0].x / 1000.0;
           Eigen::Vector2i cell_zone = world2grid(Eigen::Vector2d(zone_x,zone_y));
           if(cell_x <= cell_zone[0] || cell_x >= cell_zone[1] || cell_y <= cell_zone[0] || cell_y >=cell_zone[1])    continue;
         }*/
         if(isFree(Eigen::Vector2i(cell_x,cell_y))) 
         {
           Eigen::Vector2d tmp_pos(grid2world(Eigen::Vector2i(cell_x,cell_y)));
           float diff_x,diff_y;
           if(!flag)
           {
             diff_x = goal_pose(0) - tmp_pos(0);
             diff_y = goal_pose(1) - tmp_pos(1);
           }else
           {
             diff_x = start_pose(0) - tmp_pos(0);
             diff_y = start_pose(1) - tmp_pos(1);           
           }

           float distance = sqrt(diff_x * diff_x + diff_y*diff_y);
           if(distance < min_distance)
           { 
              //std::cout<<"valid pos x = "<<tmp_pos(0)<<" ,y = "<<tmp_pos(1)<<" ,delta_x = "<< x<<" ,delta_y = "<<y<<std::endl;
              point<<cell_x,cell_y;
              min_distance = distance;
           }
         }  
      }
    }
    if(min_distance == 10000)
    {
      return findNearPoint(point,flag,expand_num+1);
    }
    else
    {
      return true;
    }
 }

 bool JPS_Search::JPS_path_search(Eigen::Vector2d start,Eigen::Vector2d goal)
 {
     //将start和goal转换到栅格坐标系下
   Eigen::Vector2i start_index = world2grid(start);
   Eigen::Vector2i goal_index = world2grid(goal);
   if(!isVaildCell(start_index) || !isVaildCell(goal_index))   {ROS_INFO("start or goal is invaild");return false;}
    //判断start和goal是否是障碍物，以及地图是否为空
    // ROS_INFO("run to here 0");
   if(!isFree(start_index))  
   {
    // ROS_INFO("run to here 0.1");
     invalid_start = true;
     if(findNearPoint(start_index,0,2))
     {
        ROS_INFO("start is occupied,new start");
        Eigen::Vector2d start_pos = grid2world(start_index);
        ROS_INFO("new start x = %lf, y = %lf",start_pos(0),start_pos(1));
     }
    //  ROS_INFO("run to here 0.2");
   }
   if(!isFree(goal_index))   
   { 
    // ROS_INFO("run to here 0.3");
     invalid_goal = true;
     if(findNearPoint(goal_index,1,2))
     {
      //  ROS_INFO("goal is occupied,new goal");
       Eigen::Vector2d goal_pos = grid2world(goal_index);
       ROS_INFO("new goal x = %lf, y = %lf",goal_pos(0),goal_pos(1));
     }
    //  ROS_INFO("run to here 0.4");
    }
   if(map_data.empty())         
   {
     //ROS_INFO("map data is empty");
     //ROS_INFO("map data size =%d,map size = %d",map_data.size(),map_size);
     return false;
   }
  //  ROS_INFO("run to here 0.5");
  //  ROS_INFO("run to here 1");
   return jps_plan(start_index,goal_index);
    
 }

 Eigen::Vector2d JPS_Search::grid2world(Eigen::Vector2i pt_index)
 {
    Eigen::Vector2d pt; 
    pt << ((double)pt_index(0) + 0.5) *  map_resolution + map_origin_x,
          ((double)pt_index(1) + 0.5) *  map_resolution + map_origin_y;
    return pt;
 }

 Eigen::Vector2i JPS_Search::world2grid(Eigen::Vector2d pt)
 {
    // ROS_INFO("---------map width = %d,map height = %d,origin_x = %f,origin_y = %f,res = %f",map_width,map_height,map_origin_x,map_origin_y,map_resolution);
    Eigen::Vector2i pt_index;
    pt_index << (int)((pt(0) - map_origin_x) / map_resolution -0.5),
                (int)((pt(1) - map_origin_y) / map_resolution -0.5);
    return pt_index;
 }

bool JPS_Search::isFree(Eigen::Vector2i pt_index)
{
    return (map_data[map_width * pt_index(1) + pt_index(0)] == 0);
}

bool JPS_Search::isOccupied(Eigen::Vector2i pt_index)
{
    return (map_data[map_width * pt_index(1) + pt_index(0)] == 100);
}

bool JPS_Search::isUnknown(Eigen::Vector2i pt_index)
{
    return (map_data[map_width * pt_index(1) + pt_index(0)] == -1);
}

int JPS_Search::coord2Id(Eigen::Vector2i pt_index)
{
   return map_width * pt_index(1) + pt_index(0);
}

bool JPS_Search::isVaildCell(Eigen::Vector2i cell_index)
{
  if ((cell_index(0) < 0) || (cell_index(0) >= map_width) || 
      (cell_index(1) < 0) || (cell_index(1) >= map_height))  return false;
  return true;
}

Eigen::Vector2i JPS_Search::Id2coord(int id)
{
  Eigen::Vector2i pt_index;
  pt_index << id % map_width, id / map_width;
  return pt_index;
}

double JPS_Search::getHeur(Eigen::Vector2i pt_index)
{
    double diff_x = xGoal - pt_index(0);
    double diff_y = yGoal - pt_index(1);
    return diff_x + diff_y;
    // return sqrt(diff_x * diff_x + diff_y * diff_y);
}

bool JPS_Search::jps_plan(Eigen::Vector2i start_index,Eigen::Vector2i goal_index)
{
   xGoal = goal_index(0); 
   yGoal = goal_index(1);
   int start_id = coord2Id(start_index);
   int goal_id  = coord2Id(goal_index);
   StatePtr currNode_ptr = std::make_shared<State>(State(start_id, start_index(0), start_index(1), 0, 0));
   currNode_ptr->g = 0;
   currNode_ptr->h = getHeur(start_index);
   return jps_graph_search(currNode_ptr,start_id,goal_id);
}

bool JPS_Search::jps_graph_search(StatePtr& currNode_ptr, int start_id, int goal_id ,int maxExpand) 
{
  currNode_ptr->heapkey = pq_.push(currNode_ptr);
  currNode_ptr->opened = true;
  hm_[currNode_ptr->id] = currNode_ptr;
  seen_[currNode_ptr->id] = true;

  int expand_iteration = 0;
  ros::Time t1 = ros::Time::now();
  while(true)
  {
    expand_iteration++;
    // get element with smallest cost
    currNode_ptr = pq_.top(); pq_.pop();
    currNode_ptr->closed = true; // Add to closed list

    if(currNode_ptr->id == goal_id) 
    {
        // ROS_INFO("Goal Reached!!!!!!");
        // std::cout<<"jps search time cost is:"<<(ros::Time::now() - t1).toSec()<<std::endl;
        break;
    }

    //printf("expand: %d, %d\n", currNode_ptr->x, currNode_ptr->y);
    std::vector<int> succ_ids;
    std::vector<double> succ_costs;
    // Get successors
     getJpsSucc(currNode_ptr, succ_ids, succ_costs);
    //  ROS_INFO("succ_ids size is:%d",succ_ids.size());
    // Process successors
    for( int s = 0; s < (int) succ_ids.size(); s++ )
    {
      //see if we can improve the value of succstate
      StatePtr& child_ptr = hm_[succ_ids[s]];
      double tentative_gval = currNode_ptr->g + succ_costs[s];

      if( tentative_gval < child_ptr->g )
      {
        child_ptr->parentId = currNode_ptr->id;  // Assign new parent
        child_ptr->g = tentative_gval;    // Update gval

        // if currently in OPEN, update
        if( child_ptr->opened && !child_ptr->closed) 
        {
          pq_.increase( child_ptr->heapkey );       // update heap
          child_ptr->dx = (child_ptr->x - currNode_ptr->x);
          child_ptr->dy = (child_ptr->y - currNode_ptr->y);
          if(child_ptr->dx != 0)
            child_ptr->dx /= std::abs(child_ptr->dx);
          if(child_ptr->dy != 0)
            child_ptr->dy /= std::abs(child_ptr->dy);
        }
        // if currently in CLOSED
        else if( child_ptr->opened && child_ptr->closed)
        {
          // printf("ASTAR ERROR!\n");
        }
        else // new node, add to heap
        {
          child_ptr->heapkey = pq_.push(child_ptr);
          child_ptr->opened = true;
        }
      } //
    } // Process successors

    if(maxExpand > 0 && expand_iteration >= maxExpand) 
    {
        //ROS_INFO("MaxExpandStep [%d] Reached!!!!!!\n\n", maxExpand);
      //return false;
      return false;
    }

    if( pq_.empty())
    {
        //ROS_INFO("Priority queue is empty!!!!!!\n\n");
      //return false;
      return false;
    }
  }
  path_ = recoverPath(currNode_ptr, start_id);

  getPathSpecialPoint(path_);

  return true;
}

void JPS_Search::getPathSpecialPoint(std::vector<StatePtr>& path)
{
  global_path.poses.clear();
  float last_dir = 0.1;
  int path_len = path.size();
  path_length = 0;
  for(int i =0;i < path_len-1;i++)
  {
    float dir;
    Eigen::Vector2d path_node(grid2world(Eigen::Vector2i(path[i]->x,path[i]->y)));
    Eigen::Vector2d path_node_next(grid2world(Eigen::Vector2i(path[i+1]->x,path[i+1]->y)));
    if(path_node(0) == path_node_next(0))  dir = 0;
    else if(path_node(1) == path_node_next(1))   dir = 100;
    else    
       dir = (path_node(1) - path_node_next(1)) / (path_node(0) - path_node_next(0));
    if(dir != last_dir)
    {
     geometry_msgs::PoseStamped tmp_path_node;
     tmp_path_node.pose.position.x = path_node(0);
     tmp_path_node.pose.position.y = path_node(1);
     global_path.poses.insert(global_path.poses.begin(),1,tmp_path_node);
     last_dir = dir;
    }
  }
     Eigen::Vector2d path_node(grid2world(Eigen::Vector2i(path[path_len-1]->x,path[path_len-1]->y)));
     geometry_msgs::PoseStamped goal_node;
     goal_node.pose.position.x = path_node(0);
     goal_node.pose.position.y = path_node(1);
     global_path.poses.insert(global_path.poses.begin(),1,goal_node);

     if(invalid_start)            //如果给定起始点不在安全区内，重新找到安全起点并规划路径，最后还需要将给定起点添加到路径中
    {
     geometry_msgs::PoseStamped valid_start_node;
     valid_start_node.pose.position.x = start_pose(0);
     valid_start_node.pose.position.y = start_pose(1);
     global_path.poses.insert(global_path.poses.begin(),1,valid_start_node);
    }

     /*if(invalid_goal)                 //如果给定目标点是无效的，则在搜索到邻近点后，将两点连通
     {
       geometry_msgs::PoseStamped valid_goal_node;
       valid_goal_node.pose.position.x = goal_pose(0);
       valid_goal_node.pose.position.y = goal_pose(1);
       global_path.poses.push_back(valid_goal_node);
     }*/

   for(int point_index = 0;point_index<global_path.poses.size();point_index++)
   {
    //  std::cout<<"x = "<<global_path.poses[point_index].pose.position.x<<" ,y = "<<global_path.poses[point_index].pose.position.y<<std::endl;
      // ROS_INFO("global path node x = %f,y = %f",global_path.poses[point_index].pose.position.x,global_path.poses[point_index].pose.position.y);
      if(point_index > 0)         //计算路径的长度
      {
             float diff_x = global_path.poses[point_index].pose.position.x - global_path.poses[point_index-1].pose.position.x;
             float diff_y = global_path.poses[point_index].pose.position.y - global_path.poses[point_index-1].pose.position.y;
             float point_distance = sqrt(diff_x * diff_x + diff_y *diff_y);
             /*if(point_distance < 0.2) 
             {
               auto iter = global_path.poses.begin() + point_index;
               global_path.poses.erase(iter);
               point_index --;
               continue;
             }  */ 
              path_length += point_distance;  
          }   
   }
    // discretePath(global_path);
    global_path.header.stamp = ros::Time::now();
    global_path.header.frame_id = "map";
    // path_pub.publish(global_path);
} 

void JPS_Search::discretePath(nav_msgs::Path path)
{
   discrete_path.poses.clear();
   if(invalid_start)            //如果给定起始点不在安全区内，重新找到安全起点并规划路径，最后还需要将给定起点添加到路径中
    {
      geometry_msgs::PoseStamped valid_start_node;
      valid_start_node.pose.position.x = start_pose(0);
      valid_start_node.pose.position.y = start_pose(1);
      discrete_path.poses.push_back(valid_start_node);
    }
   for(int i = 0;i<path.poses.size() - 1;i++)
   {
     float node_x = path.poses[i].pose.position.x;
     float node_y = path.poses[i].pose.position.y;
     float node_x_next = path.poses[i+1].pose.position.x;
     float node_y_next = path.poses[i+1].pose.position.y;

     float distance = sqrt(pow(node_x_next - node_x,2) + pow(node_y_next - node_y,2));
     if(distance > PATH_POINT_DISTANCE)     //大于最小距离，则离散化
     {
        int point_num = (int)(distance / PATH_POINT_DISTANCE);
        for(int point_index = 0;point_index < point_num;point_index++)
        {
           float point_x = node_x + point_index *(node_x_next - node_x) / point_num;
           float point_y = node_y + point_index *(node_y_next - node_y) / point_num;
           geometry_msgs::PoseStamped tmp_path_node;
           tmp_path_node.pose.position.x = point_x;
           tmp_path_node.pose.position.y = point_y;
           discrete_path.poses.push_back(tmp_path_node);
        }
     }
   }
      geometry_msgs::PoseStamped goal_node;            //将规划路径的终点插入到新的离散化路径中
      int path_len = path.poses.size();
      goal_node.pose.position.x = path.poses[path_len-1].pose.position.x;
      goal_node.pose.position.y = path.poses[path_len-1].pose.position.y;
      discrete_path.poses.push_back(goal_node);

    if(invalid_goal)                 //如果给定目标点是无效的，则在搜索到邻近点后，将两点连通
    { 
      geometry_msgs::PoseStamped valid_goal_node;
      valid_goal_node.pose.position.x = goal_pose(0);
      valid_goal_node.pose.position.y = goal_pose(1);
      discrete_path.poses.push_back(valid_goal_node);
    }
     
     discrete_path.header.stamp = ros::Time::now();
     discrete_path.header.frame_id = "map";
    //  path_pub.publish(discrete_path);
}

std::vector<StatePtr> JPS_Search::recoverPath(StatePtr node, int start_id) 
{
  std::vector<StatePtr> path;
  path.push_back(node);
  while(node && node->id != start_id) {
    node = hm_[node->parentId];
    path.push_back(node);
  }

  return path;
}

void JPS_Search::getJpsSucc(const StatePtr& curr, std::vector<int>& succ_ids, std::vector<double>& succ_costs)
{
    const int norm1 = std::abs(curr->dx)+std::abs(curr->dy);
    int num_neib = jn2d_->nsz[norm1][0];
    int num_fneib = jn2d_->nsz[norm1][1];
    int id = (curr->dx+1)+3*(curr->dy+1);

    for( int dev = 0; dev < num_neib+num_fneib; ++dev) 
    {
      int new_x, new_y;
      int dx, dy;
      if(dev < num_neib) 
      {
        dx = jn2d_->ns[id][0][dev];
        dy = jn2d_->ns[id][1][dev];
        if(!jump(curr->x, curr->y, dx, dy, new_x, new_y)) continue;
      }
      else 
      {
        int nx = curr->x + jn2d_->f1[id][0][dev-num_neib];
        int ny = curr->y + jn2d_->f1[id][1][dev-num_neib];
        Eigen::Vector2i tempnode(nx,ny);
        //if(isOccupied(nx,ny)) 
        if(isOccupied(tempnode)) 
        {
          dx = jn2d_->f2[id][0][dev-num_neib];
          dy = jn2d_->f2[id][1][dev-num_neib];
          if(!jump(curr->x, curr->y, dx, dy, new_x, new_y)) continue;
        }
        else
          continue;
      }


      Eigen::Vector2i newnode(new_x,new_y);
      if(!isVaildCell(newnode))  continue;
      int new_id = coord2Id(newnode);
      //int new_id = coord2Id(new_x, new_y);
      if(!seen_[new_id]) {
        seen_[new_id] = true;
        hm_[new_id] = std::make_shared<State>(new_id, new_x, new_y, dx, dy);
        hm_[new_id]->h = getHeur(/*new_x, new_y*/Eigen::Vector2i(new_x,new_y));
      }

      succ_ids.push_back(new_id);
      succ_costs.push_back(std::sqrt((new_x - curr->x) * (new_x - curr->x) +
            (new_y - curr->y) * (new_y - curr->y)));
    }
}


bool JPS_Search::jump(int x, int y, int dx, int dy, int& new_x, int& new_y ) 
{
  new_x = x + dx;
  new_y = y + dy;
  Eigen::Vector2i newnode(new_x,new_y);
  if (!isVaildCell(newnode) )     return false;
  if(isOccupied(newnode))         return false;
  // if (!isFree(newnode))     return false;
  if (new_x ==  xGoal && new_y == yGoal)   return true;
  if (hasForced(new_x, new_y, dx, dy))     return true;
    
  const int id = (dx+1)+3*(dy+1);
  const int norm1 = std::abs(dx) + std::abs(dy);
  int num_neib = jn2d_->nsz[norm1][0];
  for( int k = 0; k < num_neib-1; ++k )
  {
    int new_new_x, new_new_y;
    if(jump(new_x, new_y, jn2d_->ns[id][0][k], jn2d_->ns[id][1][k],
        new_new_x, new_new_y)) return true;
  }

  return jump(new_x, new_y, dx, dy, new_x, new_y);
}



bool JPS_Search::hasForced(int x, int y, int dx, int dy) 
{
  const int id = (dx+1)+3*(dy+1);
  for( int fn = 0; fn < 2; ++fn )
  {
    int nx = x + jn2d_->f1[id][0][fn];
    int ny = y + jn2d_->f1[id][1][fn];
    Eigen::Vector2i tempnode(nx,ny);
    //if( isOccupied(nx,ny) )
    if( isOccupied(tempnode) )
      return true;
  }
  return false;
}

constexpr int JPS2DNeib::nsz[3][2];

JPS2DNeib::JPS2DNeib() {
  int id = 0;
  for(int dy = -1; dy <= 1; ++dy) {
    for(int dx = -1; dx <= 1; ++dx) {
      int norm1 = std::abs(dx) + std::abs(dy);
      for(int dev = 0; dev < nsz[norm1][0]; ++dev)
        Neib(dx,dy,norm1,dev, ns[id][0][dev], ns[id][1][dev]);
      for(int dev = 0; dev < nsz[norm1][1]; ++dev)
      {
        FNeib(dx,dy,norm1,dev,
            f1[id][0][dev],f1[id][1][dev],
            f2[id][0][dev],f2[id][1][dev]);
      }
      id ++;
    }
  }
}

/*
void JPS2DNeib::print() {
  for(int dx = -1; dx <= 1; dx++) {
    for(int dy = -1; dy <= 1; dy++) {
      int id = (dx+1)+3*(dy+1);
      printf("[dx: %d, dy: %d]-->id: %d:\n", dx, dy, id);
      for(unsigned int i = 0; i < sizeof(f1[id][0])/sizeof(f1[id][0][0]); i++)
        printf("                f1: [%d, %d]\n", f1[id][0][i], f1[id][1][i]);
    }
  }
}*/

void JPS2DNeib::Neib(int dx, int dy, int norm1, int dev, int& tx, int& ty)
{
  switch(norm1)
  {
    case 0:
      switch(dev)
      {
        case 0: tx=1; ty=0; return;
        case 1: tx=-1; ty=0; return;
        case 2: tx=0; ty=1; return;
        case 3: tx=1; ty=1; return;
        case 4: tx=-1; ty=1; return;
        case 5: tx=0; ty=-1; return;
        case 6: tx=1; ty=-1; return;
        case 7: tx=-1; ty=-1; return;
     }
    case 1:
      tx = dx; ty = dy; return;
    case 2:
      switch(dev)
      {
        case 0: tx = dx; ty = 0; return;
        case 1: tx = 0; ty = dy; return;
        case 2: tx = dx; ty = dy; return;
      }
  }
}

void JPS2DNeib::FNeib( int dx, int dy, int norm1, int dev,
                       int& fx, int& fy, int& nx, int& ny)
{
  switch(norm1)
  {
    case 1:
      switch(dev)
      {
        case 0: fx= 0; fy= 1; break;
        case 1: fx= 0; fy= -1;  break;
      }

      // switch order if different direction
      if(dx == 0)
        fx = fy, fy = 0;

      nx = dx + fx; ny = dy + fy;
      return;
    case 2:
      switch(dev)
      {
        case 0:
          fx = -dx; fy = 0;
          nx = -dx; ny = dy;
          return;
        case 1:
          fx = 0; fy = -dy;
          nx = dx; ny = -dy;
          return;
      }
  }
}

