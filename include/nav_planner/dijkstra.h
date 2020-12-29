//迪杰斯特拉算法

#ifndef _DIJKSTRA_H
#define _DIJKSTRA_H

//优先 大小
#define PRIORITYBUFSIZE 10000
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <nav_planner/planner_core.h>
#include <nav_planner/planner_core.h>

#define push_cur(n) \
{\
    if(n>=0 && n<ns_ && !pending_[n] && getCost(costs, n) < lethal_cost_ && currentEnd_ < PRIORITYBUFSIZE)\
    {\
        currentBuffer_[currentEnd_++] = n;\
        pending_[n] = true;\
    }\
}

#define push_next(n) \
{\
    if (n>=0 && n < ns_ && !pending_[n] && getCost(costs, n) < lethal_cost_ && nextEnd_ < PRIORITYBUFSIZE)\
    {\
        nextBuffer_[nextEnd_++] = n;\
        pending_[n] = true;\
    }\
}

#define push_over(n) \
{\
    if (n>=0 && n < ns_ && !pending_[n] && getCost(costs, n) < lethal_cost_ && overEnd < PRIORITYBUFSIZE)\
    {\
        overBuffer_[overEnd_++] = n;\
        pending_[n] = true;\
    }\
}

namespace nav_planner
{

class DijkstraExpansion : public Expander
{
public:
    DijkstraExpansion(Potential* p_calc, int nx, int ny);
    ~DijkstraExpansion();

    //启发函数
    bool Potential(unsigned char* costs, double start_x, double start_y, double end_x, double end_y, int cycles,
                    float* potential);
    //设置地图大小
    void setSize(int nx,int ny);
    //设置中性成本
    void setNeutralCost(unsigned char neutral_cost){
        neutral_cost_ = neutral_cost;
        priorityIncrement_ = 2 * neu
    }
    //设置精确开始
    void setPreciseStart(bool precise){precise_ = precise; }

private:
    /* 方法 */
    //更新索引为n的单元格
    // costs为代价
    //potential 为正在计算的点位阵列
    //n为要更新的索引
    void updateCell(unsigned char* costs, float potential, int n);

    //获取某点的代价
    float getCost(unsigned char* costs, int n){
        float c = costs[n];
        //如果改点代价 小于 致命代价-1, 或者 未知参数为1, 并且 该点cost等于255
        if (c < lethal_cost_ - 1 || (unknown_ && c==255))
        {
            //代价等于  当前代价 * 因数 + 中立代价
            c = c * factor_ + neutral_cost_;
            //如果结果大于致命代价,则该点等于致命代价-1
            if (c >= lethal_cost_)
                c = lethal_cost_ - 1;
            return c;
        }
        //返回致命代价
        return lethal_cost_;
    }


    /* data */
    //块优先级缓冲区
    //优先级块的存储缓冲区
    int *buffer1_, *buffer2_, *buffer3_;
    //优先级缓冲区块
    //当前缓冲区/下一个缓冲区/停止缓冲区
    int *currentBuffer_, *nextBuffer_, *overBuffer_;
    //数组的终点
    int currentEnd_, nextEnd_, overEnd_;
    //传播过程中的未决细胞(估计是没有启发过的点)
    bool *pending_;
    //准确
    bool precise_;

    //阻止优先级阈值
    //当前阈值
    float threshold_;//阈
    //优先级阈值增量
    float priorityIncrement_; //优先级 递增

};


}//end namespace nav_planner


#endif