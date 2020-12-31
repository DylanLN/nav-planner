//方向过滤器

#ifndef _ORIENTATION_FILTER_H
#define _ORIENTATION_FILTER_H
#include <nav_msgs/Path.h>

namespace nav_planner
{
//枚举
enum OrientationMode { NONE, FORWARD, INTERPOLATE, FORWARDTHENINTERPOLATE, BACKWARD, LEFTWARD, RIGHTWARD };

class OrientationFilter
{
public:
    OrientationFilter() : omode_(NONE){}

    //路径处理
    virtual void processPath(const geometry_msgs::PoseStamped& start,
                            std::vector<geometry_msgs::PoseStamped>& path);
    //基于位置导数设置角度
    void setAngleBasedOnPositionDerivative(std::vector<geometry_msgs::PoseStamped>& path, int index);
    //插
    void interpolate(std::vector<geometry_msgs::PoseStamped>& path,
                    int start_index, int end_index);
    //设置方向模式
    void setMode(OrientationMode new_mode){omode_ = new_mode;}
    void setMode(int new_mode){ setMode( (OrientationMode) new_mode); }

    //设置窗口大小
    void setWindowSize(size_t window_size){window_size_ = window_size;}


protected:
    //方向模式
    OrientationMode omode_;
    //窗口大小
    int window_size_;
};


}



#endif
