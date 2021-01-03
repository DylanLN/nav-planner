
#include <nav_planner/gradient_path.h>
#include <algorithm>
#include <stdio.h>
#include <nav_planner/planner_core.h>


namespace nav_planner
{

//构造函数
GradientPath::GradientPath(Potential* p_calc) : Traceback(p_calc), pathStep(0.5){
    gradx_ = grady_ = NULL;
}
//析构函数
GradientPath::~GradientPath(){
    if(gradx_) delete[] gradx_;
    if(grady_) delete[] grady_;
}
//设置地图的长度
void GradientPath::setSize(int xs, int ys){
    Traceback::setSize(xs, ys);
    if(gradx_) delete[] gradx_;
    if(grady_) delete[] grady_;
    //gradx_和grady_的长度都是地图面积
    gradx_ = new float[xs * ys];
    grady_ = new float[xs * ys];
}

float GradientPath::gradCell(float* potential, int n)
{
    if(gradx_[n] + grady_[n] > 0.0)
        return 1.0;
    
}


//获取路径
bool GradientPatn::getPath(float* potentoal, double start_x, double start_y, double goal_x, double gola_y,
                           std::vector<std::pair<float, float>>& path){
    std::pair<float, float> current;
    int stc = getIndex(goal_x, goal_y);

    float dx = goal_x - (int)goal_x;
    float dy = goal_y - (int)goal_y;
    //求最大面积
    int ns = xs_ * xy_;

    //将x，y队列初始化0
    memset(gradx_, 0, ns * sizeof(float));
    memset(grady_, 0, ns * sizeof(float));

    //运行次数
    int c = 0;

    while(c++ < ns * 4){
        //这个点还是没搞懂
        double nx = stc % xs_ + dx;
        double ny = stc % xs_ + dy;

        //如果当前点离起始点小于0.5，则return，因为已经找到起始点了
        if(fabs(nx - start_x) < 0.5 && fabs(ny - start_y) < 0.5){
            current.first = start_x;
            current.sencond = start_y;
            path.push_back(current);
            return true;
        }

        //如果找到了最上面，或者最下面，也就是超出地图的边界
        if(stc < xs_ || stc > xs_ * ys_ - xs_){
            ROS_INFO("[PathCalc] out of bounds");
            return false;
        }

        current.first = nx;
        current.second = ny;

        //将该点push到路径列表中
        path.push_back(current);

        //检测到震荡
        bool oscillation_detected = false;
        //求出当前怕path的长度
        int npath = path.size();

        //检测震荡，检测是否当前点与上前一个点相同
        if(npath > 2 && path[npath - 1].first == path[npath - 3].first
            && path[npath - 1].second == path[npath - 3].decond){
            ROS_INFO("[PathCalc] oscillation detected, attempting fix.")
            oscillation_detected = true;
        }

        int stcnx = stc + xs_;
        int stcpx = stc - xs_;

        //检查当前点及附近八个点的潜力值
        //是否有一个处于未遍历到的点
        if(potential[stc] >= POT_HIGH ||
                potential[stc + 1] >= POT_HIGH ||
                potential[stc - 1] >= POT_HIGH ||
                potential[stcnx] >= POT_HIGH ||
                potential[stcnx + 1] >= POT_HIGH ||
                potential[stcnx - 1] >= POT_HIGH ||
                potential[stcpx] >= POT_HIGH ||
                potential[stcpx + 1] >= POT_HIGH ||
                potential[stcpx - 1] >= POT_HIGH ||
                oscillation_detected){
            int minc = stc;
            int minp = potential[stc];
            //从最小的一个开始
            int st = stcpx -1;
            //最下层三个点
            for (int i = 0; i < 3; i++) {
                if(potential[st] < minp){
                    minp = potential[st];
                    minc = st;
                }
                st ++;
            }
            //当前点旁边的两个点（去除当前点）
            st = stc - 1;
            for (int i = 0; i < 2; i++) {
                if(potential[st] < minp){
                    minp = potential[st];
                    minc = st;
                }
                st += 2;
            }

            st = stcnx - 1;
            //最上层三个点
            for (int i = 0; i < 3; i++) {
                if(potential[st] < minp){
                    minp = potential[st];
                    minc = st;
                }
                st ++;
            }

            //stc等于潜力值最小的那个点
            stc = minc;
            dx = 0;
            dy = 0;
            //如果最小点依然是未启发过的点
            if(potential[stc] >= POT_HIGH){
                ROS_DEBUG("[PATH CALC] No path found, high potential")
                return 0;
            }
        }else{//如果渐变很好（说的是没有震荡或者未启发过得点）
            gradCell(potential, stc);
            gradCell(potential, stc + 1);
            gradCell(potential, stcnx);
            gradCell(potential, stcnx + 1);


        }

    }

}









}
