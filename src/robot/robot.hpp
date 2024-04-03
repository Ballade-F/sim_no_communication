#pragma once

#include <eigen3/Eigen/Eigen>
#include "MP_Hungarian.h"
#include "graph_searcher.hpp"
#include "map.hpp"
#include "ring_vector.hpp"


using namespace Eigen;
using std::vector;

// struct ROBOT_DATA
// {

// };

//用vector和一个位置计数器完成滑动窗口



struct ROBOT_TRACE_POINT
{
    Vector2d position;
    double time;
};
class ROBOT_ESTIMATE_STATE
{
public:
    // uint8_t robotsNum;
    // uint8_t observeSize;//maybe no need

    double pathTime;
    int targetIndex;
    RING_VECTOR<ROBOT_TRACE_POINT> trace;
    vector<vector<Vector2d>> taskPath;
    vector<double> taskCost;
    vector<double> taskProb;

    ROBOT_ESTIMATE_STATE(int task_num)
    {
        targetIndex = -1;
        taskPath = vector<vector<Vector2d>>(task_num);
        taskCost = vector<double>(task_num);
        taskProb = vector<double>(task_num);
    }


};

class ROBOT
{
public:
    
//传入
    //map 2d
    GRID_MAP<Vector2d> map;

    //task points
    std::vector<Vector2d> taskPoints;
    uint8_t taskNum;

    // //init location
    // Vector2d initPosition;

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
//TODO:换智能指针
    std::vector<ROBOT_ESTIMATE_STATE*> otherEstimation;
//规划
    gridPathFinder pathPlanner;
    vector<vector<Vector2d>> taskPath;
    vector<double> taskCost;
//决策
    HungarianAlgorithm allocation;

//flag
    bool initFlag;

//func:
    ROBOT(GRID_MAP<Vector2d> map_, std::vector<Vector2d> taskPoints_,Vector2d initPosition_, uint8_t robotsNum_);
    //获取感知模块的输出结果
    void ROBOT_GetSenseData();

    //kalman跟踪
    void ROBOT_TraceUpdata(void);

    //初始化观测，放在循环里
    bool ROBOT_Init(void);

    //match


    //estimate
    bool ROBOT_Estimate(void);

    //collision
    bool ROBOT_CollisionSolve(void);

    void ROBOT_Ctrl(void);

    //给外部使用的进程
    void ROBOT_Process(void);

};