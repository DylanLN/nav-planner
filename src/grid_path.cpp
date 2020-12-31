#include <nav_planner/grid_path.h>
#include <algorithm>
#include <stdio.h>

namespace nav_planner
{

bool GridPath::getPath(float* potential, double start_x, double start_y, double end_x, double end_y,
                        std::vector<std::pair<float, float> >& path){

    std::pair<float, float> current;
    current.first = end_x;
    current.second = end_y;

    //获取起始点的数组下标
    int start_index = getIndex(start_x, start_y);

    //将目标点压入path
    path.push_back(current);
    int c = 0;

    //如果到起始点,则get等于start的index ,退出循环
    while (getIndex(current.first, current.second) != start_index)
    {
        float min_val = 1e10;
        int min_x = 0;
        int min_y = 0;
        //就是查找前后左右四个点的潜力值
        for (int xd = -1; xd <= 1; xd++)
        {
            for (int yd = -1; yd <= 1; yd++)
            {
                //如果xd yd都等于0时跳过本次循环
                if (xd == 0 && yd == 0)
                    continue;
                int x = current.first + xd;
                int y = current.second + yd;
                //获取x,y的下标
                int index = getIndex(x,y);
                if (potential[getIndex] < min_val)
                {
                    min_val = potential[index];
                    min_x = x;
                    min_y = y;
                }
            }
        }
        //如果找到了零点,则返回false
        if (min_x == 0 && min_y == 0)
            return false;
        
        //将最小值的前后左右的最小值加入到
        current.first = min_x;
        current.second = min_y;
        path.push_back(current);

        //如果查找次数超过地图长度的四倍,则返回false
        if(c++ > ns_ * 4)
            return false;
    }
    return true;
}



}

