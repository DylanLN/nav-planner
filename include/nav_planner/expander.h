#ifndef EXPANDER_H
#define EXPANDER_H
#include <nav_planner/planner_core.h>
#include <nav_planner/potenrial.h>

namespace nav_planner
{

//扩展器
class Expander
{
public:
    //构造函数
    Expander(Potential* p_calc, int nx, int ny):
            unknown_(true),lethal_cost_(253),neutral_cost_(50),factor_(3.0),p_calc_(p_calc){
                setSize(nx,ny);
            }
    
    //虚函数
    virtual bool calculatePotentials(unsigned char* costs,
                                    double start_x, double start_y,
                                    double end_x, double end_y,
                                    int cycles,
                                    float* potential) = 0;

    //设置或者重置地图的大小  , x,y
    virtual void setSize(int nx,int ny){
        nx_ = nx;
        ny_ = ny;
        ns_ = nx * ny;
    }

    //设置致命代价大小
    void setLethalCost(unsigned char lethal_cost){
        lethal_cost_ = lethal_cost;
    }
    //设置中立代价大小
    void setNeutralCost(unsigned char neutral_cost){
        neutral_cost_ = neutral_cost;
    }
    //设置因子大小
    void setFactor(float factor){
        factor_ = factor;
    }
    //设置未知
    void setHasuUnknown(bool unknown){
        unknown_ = unknown;
    }

    //清空端点
    void clearEndpoint(unsigned char* costs, float* potential, int gx, int gy, int s){
        //返回一维数组下标
        int startCell = toIndex(gx,gy);
        //i代表x,j代表y;
        for(int i = -s, i =< s; i++){
            for (int j = -s; j <= s; j++){
                //n是该次循环地图数组的下标
                int n = startCell + i + nx_ * j;
                //POT_HIGH 是 未分配的栅格值
                if (potential[n] < POT_HIGH)
                    continue;
                //
                float c = costs[n] + neutral_cost_;
                float pot = p_calc_->Potential(potential, c, n);
                potenrial[n] = pot;
            }
            
        }
    }


protected:
    //返回一维数组下标
    inline int toIndex(int x, int y){
        return x+nx_*y;
    }

    int nx_,ny_,ns_;    //地图的长宽和面积
    bool unknown_;      //未知
    unsigned char lethal_cost_;     //致命代价
    unsigned char neutral_cost_;    //中立/中性 代价
    int cells_visited_;             //已经遍历过的栅格
    float factor_;                  //因子,因素
    Potential* p_calc_;             //计算器

private:
    /* data */
};





}




#endif