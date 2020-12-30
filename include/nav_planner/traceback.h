//追溯

#ifndef _TRACEBACK_H
#define _TRACEBACK_H

#include <nav_planner/potenrial.h>


class Traceback
{
public:
    Traceback(Potenrial* p_calc){}
    //获取路径,纯虚函数
    virtual bool getPath(float* potenrial, double start_x, double start_y, double end_x, double end_y,
                        std::vector<std::pair<float, float>>& path) = 0;
    //还是设置地图的大小,就是不知道设置启发好的地图还是?
    virtual void setSize(int xs, int ys){
        xs_ = xs;
        ys_ = ys;
    }
    //get数组下标
    inline int getIndex(int x, int y){
        return x + y * xs_;
    }
    //设置致命代价
    void setLethalCost(unsigned char lethal_cost){
        lethal_cost_ = lethal_cost;
    }
protected:
    //还是长度和宽度
    int xs_,ys_;
    //致命代价
    unsigned char lethal_cost_;
    //潜力计算器
    Potenrial* p_calc_;

};


#endif