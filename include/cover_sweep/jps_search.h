#ifndef JPS_SEARCH_
#define JPS_SEARCH_

#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/PoseStamped.h"
#include <nav_msgs/Path.h>
#include <Eigen/Eigen>
#include <boost/heap/d_ary_heap.hpp>      // boost::heap::d_ary_heap
#include <memory>                         // std::shared_ptr
#include <limits>                         // std::numeric_limits
#include <vector>                         // std::vector
#include <unordered_map>                  // std::unordered_map



#define ROBOT_RADIUS  0.12            //机器人半径，单位m
#define SAFE_DISTANCE 0.02           //机器人行走安全距离，单位m
#define PATH_POINT_DISTANCE 0.05      //离散后的路径分辨率

  extern bool isPathPlaning;
  ///Heap element comparison
  template <class T>
  struct compare_state
  {
    bool operator()(T a1, T a2) const
    {
      double f1 = a1->g + a1->h;
      double f2 = a2->g + a2->h;
      if( ( f1 >= f2 - 0.000001) && (f1 <= f2 +0.000001) )
        return a1->g < a2->g; // if equal compare gvals
      return f1 > f2;
    }
  };

  struct State; // forward declaration
  ///State pointer
  using StatePtr = std::shared_ptr<State>;
  using priorityQueue = boost::heap::d_ary_heap<StatePtr, boost::heap::mutable_<true>,
                        boost::heap::arity<2>, boost::heap::compare< compare_state<StatePtr> >>;
///Node of the graph in graph search
  struct State
  {
    /// ID
    int id;
    /// Coord
    int x, y = 0;
    /// direction
    int dx, dy;                            // discrete coordinates of this node
    /// id of predicessors
    int parentId = -1;

    /// pointer to heap location
    priorityQueue::handle_type heapkey;

    /// g cost
    double g = std::numeric_limits<double>::infinity();
    /// heuristic cost
    double h;
    /// if has been opened
    bool opened = false;
    /// if has been closed
    bool closed = false;

    /// 2D constructor
    State(int id, int x, int y, int dx, int dy )
      : id(id), x(x), y(y), dx(dx), dy(dy)
    {}

  };

  ///Search and prune neighbors for JPS 2D
  struct JPS2DNeib {
    // for each (dx,dy) these contain:
    //    ns: neighbors that are always added
    //    f1: forced neighbors to check
    //    f2: neighbors to add if f1 is forced
    int ns[9][2][8];
    int f1[9][2][2];
    int f2[9][2][2];
    // nsz contains the number of neighbors for the four different types of moves:
    // no move (norm 0):        8 neighbors always added
    //                          0 forced neighbors to check (never happens)
    //                          0 neighbors to add if forced (never happens)
    // straight (norm 1):       1 neighbor always added
    //                          2 forced neighbors to check
    //                          2 neighbors to add if forced
    // diagonal (norm sqrt(2)): 3 neighbors always added
    //                          2 forced neighbors to check
    //                          2 neighbors to add if forced
    static constexpr int nsz[3][2] = {{8, 0}, {1, 2}, {3, 2}};

    //void print();
    JPS2DNeib();
    private:
    void Neib(int dx, int dy, int norm1, int dev, int& tx, int& ty);
    void FNeib(int dx, int dy, int norm1, int dev,
        int& fx, int& fy, int& nx, int& ny);
  };

class JPS_Search
{
public:
JPS_Search();
~JPS_Search();

   bool globalPathPlaning(Eigen::Vector2d _current_pose,Eigen::Vector2d _goal_pose,nav_msgs::OccupancyGrid& map);
   bool tryStaightPath(Eigen::Vector2d _current_pose,Eigen::Vector2d _goal_pose,nav_msgs::OccupancyGrid& map);
   void JPS_init(Eigen::Vector2d _current_pose,Eigen::Vector2d _goal_pose,nav_msgs::OccupancyGrid& map);
   void mapDataInit(nav_msgs::OccupancyGrid& map);
   bool JPS_path_search(Eigen::Vector2d start,Eigen::Vector2d goal);
   Eigen::Vector2d grid2world(Eigen::Vector2i pt_index);
   Eigen::Vector2i world2grid(Eigen::Vector2d pt);
   bool isFree(Eigen::Vector2i pt_index);
   bool isOccupied(Eigen::Vector2i pt_index);
   bool isUnknown(Eigen::Vector2i pt_index);
   bool findNearPoint(Eigen::Vector2i& point,int flag,int expand_num);
   bool jps_plan(Eigen::Vector2i start_index,Eigen::Vector2i goal_index);
   int coord2Id(Eigen::Vector2i pt_index);
   Eigen::Vector2i Id2coord(int id);
   double getHeur(Eigen::Vector2i pt_index);
   bool jps_graph_search(StatePtr& currNode_ptr, int start_id, int goal_id,int maxExpand = 50000);
   void getJpsSucc(const StatePtr& curr, std::vector<int>& succ_ids, std::vector<double>& succ_costs);
   bool jump(int x, int y, int dx, int dy, int& new_x, int& new_y );
   bool hasForced(int x, int y, int dx, int dy); 
   std::vector<StatePtr> recoverPath(StatePtr node, int start_id);
   bool isVaildCell(Eigen::Vector2i cell_index);
   void getPathSpecialPoint(std::vector<StatePtr>& path);
   void discretePath(nav_msgs::Path path);
   nav_msgs::Path getGlobalPath()  {return global_path;}
   float getPathLenght()           {return path_length;}
   nav_msgs::Path getDiscretePath() {return discrete_path;}
   void expandMap(nav_msgs::OccupancyGrid map,nav_msgs::OccupancyGrid& expand_map,int occupied_value =100);
   void expandMapCell(int cell_num,Eigen::Vector2i cell,std::vector<int>& map_data,nav_msgs::OccupancyGrid& map,int occupied_value);
   bool isOccupied(Eigen::Vector2i& cell,nav_msgs::OccupancyGrid& map,int occupied_value = 100);
   bool isValidCell(Eigen::Vector2i& cell,nav_msgs::OccupancyGrid& map);
   bool collisionCheck(Eigen::Vector2d start,Eigen::Vector2d end,nav_msgs::OccupancyGrid& map);


private:
  nav_msgs::OccupancyGrid expand_map;
  nav_msgs::Path  global_path;
  nav_msgs::Path  discrete_path;
  int map_size ;
  int map_width;
  int map_height;
  double map_origin_x;
  double map_origin_y ;
  double map_resolution;
  std::vector<unsigned char> map_data;
  float path_length;

  int xGoal, yGoal;
  priorityQueue pq_;
  std::vector<StatePtr> hm_;
  std::vector<bool> seen_;

  std::vector<StatePtr> path_;

  std::vector<std::vector<int>> ns_;
  std::shared_ptr<JPS2DNeib> jn2d_;
  
  Eigen::Vector2i goalIdx;
  Eigen::Vector2d start_pose;
  Eigen::Vector2d goal_pose;

  bool invalid_start = false;
  bool invalid_goal = false;
};

#endif
