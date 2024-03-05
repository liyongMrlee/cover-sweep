# cover-sweep
清洁机器人全覆盖路径规划算法

功能：
1、实现清洁机器人全覆盖清扫
2、依据沿边的轨迹，规划清扫路径
3、依据沿边轨迹的形状，自动进行横向清扫或纵向清扫

依赖包
map_server	加载地图
amcl		给予地图的重定位
turtlebot3包	(可用其他替代，主要用到其中的gazebo环境提供机器人仿真平台)

下载与编译方法
mkdir robot_ws/src
cd robot_ws/src
git clone https://github.com/liyongMrlee/cover-sweep
catkin_make 





