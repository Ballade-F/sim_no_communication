#pragma once

#include <Eigen/Eigen>
#include "MP_Hungarian.h"
#include "graph_searcher.hpp"
#include "map.hpp"
#include "ring_vector.hpp"
#include "extend_kalman.hpp"

using namespace Eigen;
using std::vector;

// struct ROBOT_DATA
// {

// };

//用vector和一个位置计数器完成滑动窗口

inline double robot_distanceL2(Vector2d x1, Vector2d x2);

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
    double distance;//trace中最旧的点到算path点的距离
    int targetIndex;
    RING_VECTOR<ROBOT_TRACE_POINT> trace;
    vector<vector<Vector2d>> taskPath;
    vector<int> taskPathLastIndex;
    vector<RING_VECTOR<Vector2d>> estimateTrace;
    vector<double> taskCost;
    vector<double> taskProb;

    ExtendKalman kalman;

    ROBOT_ESTIMATE_STATE(int task_num,Eigen::Matrix2d Q_, Eigen::Matrix2d R_):kalman(Q_,R_)
    {
        targetIndex = -1;
        pathTime = 0;
        distance = 0;
        taskPath = vector<vector<Vector2d>>(task_num);
        estimateTrace = vector<RING_VECTOR<Vector2d>>(task_num);
        taskPathLastIndex = vector<int>(task_num);
        taskCost = vector<double>(task_num);
        taskProb = vector<double>(task_num);

        //TODO:trace.push_back(init_point);

    }
    inline void ESTIMATE_MatchPathTrace(void);
    inline void ESTIMATE_CalculateProb(void);
    inline void ESTIMATE_EstimateTraceUpdate(ROBOT_TRACE_POINT new_trace);

    //TODO:每次update的时候用滑窗，不用全计算，只有重规划时需要全计算。distance需要动态调整

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

    //observe time
    double dt = 0.1;

    double timeStamp;

//性能相关配置参数,大于等于2
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
    void ROBOT_TraceUpdata(double t);

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