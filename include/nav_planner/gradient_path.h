//get路径,使用梯度下降的方式.(其实就是加上对角)
#ifndef _GRADIENT_PATH_H
#define _GRADIENT_PATH_H
//追溯
#include <nav_planner/traceback.h>

#include <math.h>

//这个估计是路径优化的
namespace nav_planner
{
class GradientPath : public Traceback
{
public:
    GradientPath(arguments);
    ~GradientPath();


    //设置地图的长度
    void setSize(int xs, int ys);

    //路径构造
    bool getPath(float* potential, double start_x, double start_y, double end_x, double end_y,
                std::std::vector<std::pair<float, float> >& path);

private:
    //取得最近点
    //round函数四舍五入
    inline int getNearestPoint(int stc, float dx, float dy){
        int pt = stc + (int)round(dx) + (int)(xs_ * round(dy));
    }

    float gradCell(float* potential, int n);
    //梯度阵列，势阵列的大小
    float *gradx_, *grady_;

    //跟随梯度的步长
    float pathStep;

};
}
#endif