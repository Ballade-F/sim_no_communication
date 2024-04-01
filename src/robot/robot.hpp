#pragma once

#include <eigen3/Eigen/Eigen>
#include "MP_Hungarian.h"
#include "graph_searcher.hpp"
#include "map.hpp"


using namespace Eigen;
using std::vector;

// struct ROBOT_DATA
// {

// };

class ROBOT_ESTIMATE_STATE
{
    uint8_t robotsNum;
    uint8_t observeSize;
    vector<Vector2d> trace;
    vector<vector<Vector2d>> taskPath;
    vector<double> taskProb;

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

//性能相关配置参数
    uint8_t observeSize;
//感知
//TODO:用sophus
    Vector2d selfPosition;
    double selfRotation;
    std::vector<Vector2d> otherPosition;
    // std::vector<double> otherRotation;

    std::vector<int8_t> taskState;

//估计
    std::vector<ROBOT_ESTIMATE_STATE*> otherEstimation;
//规划
    gridPathFinder pathPlanner;
//决策
    HungarianAlgorithm allocation;

//flag
    bool initFlag;

//func:
    ROBOT(GRID_MAP<Vector2d> map_, std::vector<Vector2d> taskPoints_, uint8_t robotsNum_):map(map_),taskPoints(taskPoints_),robotsNum(robotsNum_)
    {
        initFlag = false;
    }
    //获取感知模块的输出结果
    bool ROBOT_GetSenseData();

    //初始化观测，放在循环里
    bool ROBOT_Init(void);

    //match


    //estimate
    bool ROBOT_Estimate(void);

    //collision
    bool ROBOT_CollisionSolve(void);

    bool ROBOT_Ctrl(void);

    //给外部使用的进程
    void ROBOT_Process(void);

};