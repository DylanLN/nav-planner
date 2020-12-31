
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

//获取路径










}
