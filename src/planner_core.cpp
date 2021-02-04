

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
    //调用初始化函数2
    //将 Costmap2DROS 转化位 costmap 提取其 frameid
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

        //是否使用navfn的方式  no
        convert_offset_ = 0.5;

        //是否使用二次近似函数    no
        p_calc_ = new Potential(cx, cy);

        //使用地杰斯特拉算法 yes
        DijkstraExpansion* de = new DijkstraExpansion(p_calc_, cx, cy);
            //是否使用老旧的计算方式 no
            de->setPreciseStart(true);
        planner_ = de;

        //延边界创建路径 yes  使用梯度下降 no
        path_marker_ = new GridPath(p_calc_);

        //给路径加方向
        orientation_filter_ = new orientationFilter();

        //发布消息
        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan",1);
        potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential",1);

        //允许规划 未知（unknow）中的全局路径 no
        planner_->setHasUnknown(false);

        //设置允许目标点误差范围
        default_tolerance_ = 0.0；

        //获取tf 的prefix(字首)
        ros::NodeHandle prefix_nh;
        tf_prefix = tf:: getPrefixParam(prefix_nh);

        //发布make plan服务
        make_plan_src_ = private_nh.advertiseService("make_plan", &GlobalPlanner::makePlanService, this);

        //动态调参 略去

        initialized_ = true;
    }else{
        //这个已经被初始化，您不能调用两次
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
    }
}

//将某个costmap设置为free
void GlobalPlanner::clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my)
{
    if (initialized_){
        costmap->setCost(mx, my, costmap_2d::FREE_SPACE);
    }else{
        ROS_ERROR("this planner has not been initialized yet");
    }
}

//makeplan 回调函数
bool GlobalPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res)
{
    //调用makeplan函数
    makePlan(req.start, req.goal, res.plan.poses);

    //赋予时间及frame id
    res.plan.header.stamp = ros::Time::now();
    res.plan.header.frame_id = frame_id_;

    return true;
}

//将map上的坐标系转化为world上的坐标系
void GlobalPlanner::mapToWorld(double mx, double my, double& wx, double& wy)
{
    wx = costmap_->getOriginX() + (mx + convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my + convert_offset_) * cosrmap_->getResolution();
}

//将world上的坐标转化为map上的坐标
bool GlobalPlanner::worldToMap(double wx, double wy, double& mx, double& my)
{
    //获得地图的长宽（栅格地图）
    double origin_x = costmap_->getOriginX();
    double origin_y = costmap_->getOriginY();

    double resolution = costmap_->getResolution();

    //如果超出范围
    if (wx < origin_x || wy < origin_y)
        return false;
    // x = (当前点 - 长度)/分辨率  - 转换偏移
    mx = (wx - origin_x) / resolution - convert_offset_;

    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                             const geometry_msgs::PoseStamped& goal,
                             std::vector<geometry_msgs::PoseStamped>& plan)
{
    return makePlan(start, goal, default_tolerance_, plan);
}

//主要函数：路径规划函数
//start 起始点的位资
//goal 目标点的位资
//telerance 目标点的允许误差
//plan 路径
bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                             const geometry_msgs::PoseStamped& goal,
                             double tolerance,
                             std::vector<geometry_msgs::PoseStamped>& plan)
{
    

}

}


