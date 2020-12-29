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
    if (precise_)
    {
        double dx = start_x - (int)start_x;
        double dy = start_y - (int)start_y;

        dx = floorf(dx * 100 + 0.5) / 100;
    }
    

}



}
