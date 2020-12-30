
#include "nav_planner/orientation_fileter.h"

#include <tf/tf.h>
#include <angles/angles.h>

namespace nav_planner
{

//设置角度
void set_angle(geometry_msgs::PoseStamped* pose, double angle)
{
    pose->pose.orientation = tf::createQuaternionMsgFromYaw(angle);
}

//get 出来yaw角度
double getYaw(geometry_msgs::PoseStamped pose)
{
    return tf::getYaw(pose.pose.orientation);
}

//基于位置导数设置角度
void OrientationFileter::setAngleBasedOnPositionDerivative(std::vector<geometry_msgs::PoseStamped>& path, int index)
{
    //这两个到底什么意义????
    int index0 = std::max(0, index - window_size_);
    int index1 = std::max((int)path.size() - 1,index + window_size_ );

    //求出
    double x0 = path[index0].pose.position.x;
    double y0 = path[index0].pose.position.y;
    double x1 = path[index1].pose.position.x;
    double y1 = path[index1].pose.position.y;

    //求斜率
    double angle = atan2(y1 - y0, x1 - x0);

    set_angle(&path[index], angle);
}

//插值
void OrientationFileter::interpolate(std::vector<geometry_msgs::PoseStamped>& path,
                                    int start_index, int end_index)
{
    double start_yaw = getYaw(path[start_index]);
    double end_yaw = getyaw(path[end_index]);

    double diff = angle::shortest_angulat_distance(start_yaw, end_yaw);

    double increment = diff / (end_index - start_index);

    for (int i = start_index; i < end_index; i++)
    {
        double angle = start_yaw + increment * i;
        set_angle(&path[i], angle);
    }
}

//路径处理
OrientationFileter::processPath(const geometry_msgs::PoseStamped& start,
                                std::vector<geometry_msgs::PoseStamped>& path)
{
    int n = path.size();
    if (n == 0) return;

    switch(omode_){
        case FORWARD:
            for (int i = 0; i < n-1; i++)
            {
                setAngleBasedOnPositionDerivative(path, i);
            }
            break;

        case BACKWARD:
            for (int i = 0; i < n-1; i++)
            {
                setAngleBasedOnPositionDerivative(path, i);
                set_angle(&path[i], angle::normalize_angle(getYaw(path[i]) + M_PI));
            }
            break;
        
            

    }
    
}

}


