//迪杰斯特拉算法

#include <nav_planner/dijkstra.h>

//算法的头文件
#include <algorithm>

namespace nav_planner{

//构造函数
DijkstraExpansion::DijkstraExpansion(Potential* p_calc, int nx, int ny) :
                    Expander(p_calc, nx, ny), pending_(NULL), precise_(false){
                        //初始化优先级 buf 长度为PRIORITYBUFSIZE = 10000;
                        buffer1_ = new int[PRIORITYBUFSIZE];
                        buffer2_ = new int[PRIORITYBUFSIZE];
                        buffer3_ = new int[PRIORITYBUFSIZE];

                        priorityIncrement_ = 2 * neutral_cost_;
}

//析构函数
DijkstraExpansion::~DijkstraExpansion(){
    delete[] buffer1_;
    delete[] buffer2_;
    delete[] buffer3_;

    if (pending_)
        delete[] pending_;
}

//设置或初始化地图的长度
void DijkstraExpansion::setSize(int xs, int ys){
    Expander::setSize(xs,ys);
    
    if (pending_)
        delete[] pending_;
    
    pending_ = new bool[ns_];
    //初始化未决全部设置为0
    memset(pending_, 0, ns_ * sizeof(bool));
}

//迪杰斯特拉主要的启发函数
//广度优先
//
bool DijkstraExpansion::Potential(unsigned char* costs, double start_x, double start_y, double end_x,
                                    double end_y, int cycles, float* potential){

    //设置已经遍历过的栅格为0
    cells_visited_ = 0;
    //阈值设置为致命代价
    threshold_ = lethal_cost_;
    //将buffer1的地址传递给当前缓冲区
    currentBuffer_ = buffer1_;
    //当前缓冲区的长度设置为0
    currentEnd_ = 0;
    //把第二个缓冲区给下一个缓冲区
    nextBuffer_ = buffer2_;
    nextEnd_ = 0;
    overBuffer_ = buffer3_;
    overEnd_ = 0;
    //初始化未决全部设置为0
    memset(pending_, 0, ns_ * sizeof(bool));
    //初始化 potential里面的值全部为 POT_HIGH = 10e10 10的10次方
    std::fill(potential, potential + ns_, POT_HIGH);

    //返回开始点的一维数组下标
    int k = toIndex(start_x, start_y);

    //判断准确值
    //使用true新的方式 还是 false:老版navfn启发方式
    if (precise_)
    {
        double dx = start_x - (int)start_x;
        double dy = start_y - (int)start_y;

        //向下取整 //保留小数点后两位
        dx = floorf(dx * 100 + 0.5) / 100;
        dy = floorf(dy * 100 + 0.5) / 100;

        //起始点代价等于  中立代价 *2 * dx * dy  不太明白为啥乘dxdy
        potential[k] = neutral_cost_ * 2 * dx * dy;
        potential[k-1] = neutral_cost_ * 2 * (1-dx) * dy;
        potential[k+nx_] = neutral_cost_ * 2 * dx * (1-dy);
        potential[k+nx_+1] = neutral_cost_ * 2 * (1-dx) * (1-dy);

        //把k附近的点全部压入 current队列
        push_cur(k+2);
        push_cur(k-1);
        push_cur(k+nx_-1);
        push_cur(k+nx_+2);

        push_cur(k-nx_);
        push_cur(k-nx_+1);
        push_cur(k + nx_*2);
        push_cur(k + nx_*2 + 1);

    }else{
        //当前点的代价设为0
        potential[k] = 0;
        //把前后左右全部压入current队列
        push_cur[k+1];
        push_cur[k-1];
        push_cur[k-nx_];
        push_cur[k+nx_];
    }
    
    //最大优先级块大小
    int nwv = 0;
    //放入优先块的信元数量(放进去的栅格的数量)
    int nc = 0;

    //设置开始单元(分明是关闭好吗)//就是目标点
    int startCell = toIndex(end_x, end_y);

    //进行多次循环，除非被打断,或者循环
    int cycle = 0;
    for (; cycle < cycles; cycle++)
    {
        //优先权块为空
        if (currentEnd_ == 0 && nextEnd_ == 0)
            return false;
        
        //统计资料
        nc += currentEnd_;
        //最大优先级块大小 不能小于 当前队列大小
        if (currentEnd_ > nwv)
            nwv = currentEnd_;
        
        //在当前优先级缓冲区上重置未决标志
        //吧当前缓冲区指针 给pb
        int *pb = currentBuffer_;
        int i = currentEnd_;
        //把当前队列的未决全部设置为 false
        while (i-- > 0)
            pending_[*(pb++)] = false;

        //处理当前优先级缓冲区
        pb = currentBuffer_;
        i = currentEnd_;

        while(i-- > 0)
            updateCell(costs, potential, *pb++);
        
        //交换优先级块currentBuffer_ <=> nextBuffer_
        currentEnd_ = nextEnd_;
        nextEnd_ = 0;
        pb = currentBuffer_;
        currentBuffer_ = nextBuffer_;
        nextBuffer_ = pb;

        //查看是否已完成此优先级
        if (currentEnd_ == 0)
        {
            //递增优先级阈值
            threshold_ += priorityIncrement_;
            //将当前设置为溢出块
            currentEnd_ = overEnd_;
            overEnd_ = 0;
            pb = currentBuffer_;
            currentBuffer_ = overBuffer_;
            overBuffer_ = pb;
            //交换over 和当前队列
        }

        if (potential[startCell] < POT_HIGH)
            break;
    }
    if (cycle < cycles)
        return true;
    else
        return false;
}


//关键函数: 根据邻居的值计算单元更新后的潜在值
//从四个网格中的两个最低相邻点进行平面更新计算
//内插值的二次近似
//这里没有边界检查,函数应该很快

//二分之根号二
#define INVSQRT2 0.707106781

inline void DijkstraExpansion::updateCell(unsigned char* costs, float* potential, int n){
    //已经遍历过的栅格 自加1
    cells_visited_++;

    //做平面更新
    float c = getCost(costs, n);

    //大于致命代价,不要启发到障碍里面去
    if (c >= lethal_cost_)
        return;
    //pot  =  前后左右最小的potential + 当前的costs 
    float pot = p_calc_->Potential(potential, c, n);

    //现在将受影响的邻居添加到优先级块

    if (pot < potential[n])
    {
        //获得上下左右的 cost值, 但是不明白为啥要乘以二分之根号二
        float le = INVSQRT2 * (float)getCost(costs, n-1);
        float re = INVSQRT2 * (float)getCost(costs, n+1);
        float ue = INVSQRT2 * (float)getCost(costs, n-nx_);
        float de = INVSQRT2 * (float)getCost(costs, n+nx_);

        //potential -= 1;  不知道为啥自减一
        potential[n] =pot;

        //低成本缓冲块,暂时不清楚干啥的
        //如果当前 pot小于阈值
        if (pot < threshold_)
        {
            //如果当前点的潜力  大于 潜力-1 + cost*根号二/2
            //将其push到next的队列
            if (potential[n-1] > pot + le) push_next(n-1);
            if (potential[n+1] > pot + re) push_next(n+1);
            if (potential[n-nx_] > pot + ue) push_next(n-nx_);
            if (potential[n+nx_] > pot + de) push_next(n+nx_);
        }else{
            //如果当前点的潜力  大于 潜力-1 + cost*根号二/2
            //将其push到over的队列
            if (potential[n-1] > pot + le) push_over(n-1);
            if (potential[n+1] > pot + re) push_over(n+1);
            if (potential[n-nx_] > pot + ue) push_over(n-nx_);
            if (potential[n+nx_] > pot + de) push_over(n+nx_);
        }
    }
}


}
