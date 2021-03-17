#ifndef POTENTIAL_H
#define POTENTIAL_H
//
//潜力计算器
class PotentialCalculator
{
public:
    PotentialCalculator(int nx,int ny){
        setSize(nx,ny);
    }
    //计算方式
    virtual float calculatePotential(float* potential, unsigned char cost, int n, float prev_potential = -1){
        if (prev_potential < 0)
        {
        	//分别求出前后、左右最小的代价值
            float min_h = std::min(potential[n-1] , potential[n+1]);
            float min_v = std::min(potential[n-nx_] , potential[n+nx]);
            //从最小值中再取最小，也就是四个最小的
            prev_potential = std::min(min_h , min_v);
        }
        //返回最小的代价值+当前已经累计的cost
        return prev_potential + cost;
    }
    //设置或重置地图的大小
    virtual void setSize(int x,int y){
        nx_ = nx;
        ny_ = ny;
        ns_ = nx*ny;
    }


protected:
        //求出地图坐标点 x,y 在一维数组里面的下标
        inline int toIndex(int x,int y){
            return x + nx_*y;
        }

        int nx_ , ny_ , ns_ ; //地图的长度

};


#endif