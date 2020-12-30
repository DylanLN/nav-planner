#ifndef _PLANNERCORE_H
#define _PLANNERCORE_H

#define POT_HIGH 10e10

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <dynamic_reconfigure/server.h>
//潜力计算器
#include <nav_planner/potential.h>
//扩展器
#include <nav_planner/expander.h>
//追溯
#include <nav_planner/traceback.h>
//将GlobalPlanner注册为BaseGlobalPlanner插件

namespace nav_planner
{

class Expander;
class GridPath;









}



#endif