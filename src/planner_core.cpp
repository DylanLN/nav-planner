

#include <nav_planner/planner_core.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include <nav_planner/dijkstra.h>
#include <nav_planner/expander.h>
#include <nav_planner/gradient_path.h>
#include <nav_planner/orientation_filter.h>
#include <nav_planner/planner_core.h>
#include <nav_planner/potential.h>
#include <nav_planner/traceback.h>

PLUGINLIB_EXPORT_CLASS(nav_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

namespace nav_planner
{

//将地图外轮廓设为障碍
void GlobalPlanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value)
{
    unsigned char* pc = costarr;
    //第一行
    for (size_t i = 0; i < nx; i++)
        *pc++ = value;
    //最后一行
    pc = costarr + (ny - i) * nx;
    for (size_t i = 0; i < nx; i++)
        *pc++ = value;
    //第一列
    pc = costarr;
    for (size_t i = 0; i < ny; i++, pc += nx)
        *pc = value;
    //最后一列
    pc = costarr + nx - 1;
    for (size_t i = 0; i < ny; i++, pc += nx)
        *pc = value; 
}

//构造函数
GlobalPlanner::GlobalPlanner():costmap_(NULL),initialized_(false),allow_unknown_(true)
{}

//构造函数1
GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id)
                            : costmap_(NULL), initialized_(false), allow_unknown_(true)
{
                            //调用初始化函数
                            initialize(name, costmap, framer_id);
}

//析构函数
GlobalPlanner::~GlobalPlanner()
{
    if(p_calc_)
        delete p_calc_;
    if(planner_)
        delete planner_;
    if(path_marker_)
        delete path_marker_;
    if(dsrv_)
        delete dsrv_;
}

//初始化函数1
void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id)
{
    //判断是否已经初始化完毕
    if (!initialized_)
    {
        //获取参数 节点句柄
        ros::NodeHandle private_nh("~/"+name);
        costmap_ = costmap;
        frame_id_ = frame_id;

        //获得代价地图的长和宽
        unsigned int cx = costmap->getSizeInCellX();
        unsigned int cy = costmap->getSizeInCellY();

        private_nh.param();
    }
    
}




}


