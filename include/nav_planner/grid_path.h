//从潜力图中查找路径

#ifndef _GRID_PATH_H
#define _GRID_PATH_H

#include<vector>
#include<nav_planner/traceback.h>

namespace nav_planner
{

class GridPath : public Traceback
{
public:
    GridPath(Potential* p_calc) : Traceback(p_calc){};

    //get出来路径
    bool getPath(float* potential, double start_x, double start_y, double end_x, double end_y,
                std::vector<std::pair<float, float> >& path);

private:
    /* data */
};




}


#endif