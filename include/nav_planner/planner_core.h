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
//给路径加方向
#include <lnglobal_planner/orientation_filter.h>


namespace nav_planner
{

class Expander;
class GridPath;

//
//global_planner规划器提供ROS包装器，该包装器在成本图上运行快速的内插导航功能。

class GlobalPlanner : public nav_core::BaseGlobalPlanner{

public:
    //构造函数
    GlobalPlanner();
    GlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);
    //析构函数
    ~GlobalPlanner();


    //初始化函数
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    void initialize(std::string name, costmap_2d::Costmap2D* costmap, std:: string frame_id);


    //makeplan 函数,服务回调就是调用这个;
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan);
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance,
                    std::vector<geometry_msgs::PoseStamped>& plan);
    
    //从给定的世界点开始计算地图的完整功能
    //计算潜力
    bool computePotentialCalculator(const geometry_msgs::Point& world_point);
    //在已经计算出起点的可能性之后，将计划计算为目标（注意：您应该首先调用computePotentialCalculator）
    //从潜力中获取计划
    //估计是从潜力计算图中获取路径
    bool getPlanFromPotentialCalculator(double start_x, double start_y, double end_x, double end_y,
                                const geometry_msgs::PoseStamped& goal
                                std::vector<geometry_msgs::PoseStamped>& paln);
    //获取世界上给定点的潜力或导航成本（注意：您应该先调用“computePotentialCalculator-计算潜力”）
    //应该是获取改点的潜力
    double getPointPotentialCalculator(const geometry_msgs::Point& world_point);

    //在世界上的给定点检查有效的潜在值（注意：您应该先调用computePotentialCalculator）
    bool validPointPotentialCalculator(const geometry_msgs::Point& world_point);
    //tolerance-公差
    bool validPointPotentialCalculator(const geometry_msgs::Point& world_point, double tolerance);

    //发布路径
    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
    //makeplan回调函数
    void makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res);

protected:
    //将当前代价地图存储在这里 由makeplan调用
    costmap_2d::Costmap2D* costmap_;
    std::string frame_id;
    ros::Publisher plan_pub_;
    bool initialized_,allow_unknown_;

private:
    //ros的一些发布者等
    //发布启发过的地图
    ros::Publisher potential_pub_;
    //makeplan
    ros::ServiceServer make_plan_srv_;

    //将map坐标转化为世界坐标
    void mapToWorld(double mx, double my, double& wx, double& wy);
    //将世界坐标转化为map坐标
    bool worldToMap(double wx, double wy, double& mx, double& xy);
    //应该是清空栅格的代价值,或者潜力值
    void clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my);
    //
    void publishPotentialCalculator(float* potential);

    double planner_window_x_, planner_window_y_, default_tolerance;
    std::string tf_prefix_;
    boost::mutex mutex_;

    PotentialCalculator* p_calc_;
    Expander* planner_;
    Traceback* path_marker_;
    OrientationFilter* orientation_filter_;

    //启发过的地图相关数据
    bool publish_potential_;
    int publisher_scale_;

    void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);
    unsigned char* cost_array_;
    float* potential_array_;
    unsigned int start_x_, start_y_, end_x_, end_y_;

    //是否用老版的navfn的计算方式
    bool old_navfn_behavior_;
    float convert_offset_;

    //动态调参的相关,
    // dynamic_reconfigure::Server<lnglobal_planner::GlobalPlannerConfig> *dsrv_;
    // void reconfigureCB(lnglobal_planner::GlobalPlannerConfig &config, uint32_t level);


};

}

#endif