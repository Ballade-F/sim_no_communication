#pragma once

#include <eigen3/Eigen/Eigen>
#include "MP_Hungarian.h"
#include "graph_searcher.hpp"
#include "map.hpp"


using namespace Eigen;

struct ROBOT_DATA
{

};

class ROBOT_ESTIMATE_STATE
{

};

class ROBOT
{
public:
    
//传入
    //map 2d
    GRID_MAP<Vector2d> map;

    //task points
    std::vector<Vector2d> taskPoints;

    //total robots num
    uint8_t robotsNum;

//感知
//TODO:用sophus
    Vector2d selfPosition;
    double selfRotation;
    std::vector<Vector2d> otherPosition;
    std::vector<double> otherRotation;

    std::vector<int8_t> taskState;

//估计
    std::vector<ROBOT_ESTIMATE_STATE> otherEstimation;
//规划
    gridPathFinder pathPlanner;
//决策
    HungarianAlgorithm allocation;

    
};