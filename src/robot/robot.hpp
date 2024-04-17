#pragma once

#include <Eigen/Eigen>
#include "MP_Hungarian.h"
#include "graph_searcher.hpp"
#include "map.hpp"
#include "ring_vector.hpp"
#include "extend_kalman.hpp"

using namespace Eigen;
using std::vector;
using std::cout;
using std::endl;

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

    double pathTime;//path的时间
    double distance;//trace中最旧的点到算path点的距离
    int targetIndex;
    RING_VECTOR<ROBOT_TRACE_POINT> trace;
    vector<vector<Vector2d>> taskPath;
    // vector<int> taskPathLastIndex;
    // vector<RING_VECTOR<Vector2d>> estimateTrace;
    // vector<double> matchDis;
    vector<Vector2d> estPoint;
    vector<double> taskCost;
    vector<double> taskProb;
    double probThreshold;


    ExtendKalman kalman;

    ROBOT_ESTIMATE_STATE(int task_num,double prob_threshold,Vector2d init_pos,Eigen::Matrix2d Q_, Eigen::Matrix2d R_)
                        :probThreshold(prob_threshold),kalman(Q_,R_)
    {
        targetIndex = -1;
        pathTime = -1;
        distance = 0;
        taskPath = vector<vector<Vector2d>>(task_num);
        // estimateTrace = vector<RING_VECTOR<Vector2d>>(task_num);
        
        // matchDis = vector<double>(task_num);
        estPoint = vector<Vector2d>(task_num);
        // taskPathLastIndex = vector<int>(task_num,0);
        taskCost = vector<double>(task_num);
        taskProb = vector<double>(task_num);

        ROBOT_TRACE_POINT init_point = {init_pos,0};
        trace.push_back(init_point);

    }
    // inline void ESTIMATE_matchPathTrace(void);
    // inline void ESTIMATE_calculateProb(void);
    // inline void ESTIMATE_ProbUpdate(ROBOT_TRACE_POINT old_trace);
    inline void ESTIMATE_JudgeTarget(const vector<Vector2d> &task_points, gridPathFinder &path_planner);
    inline Vector2d ESTIMATE_EstObserve(Vector2d new_point);
    inline void ESTIMATE_estUpdate(void);
    inline void ESTIMATE_probUpdate(void);
    inline Vector2d ESTIMATE_findEst(int path_idx, double trace_dis);



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

    //task reallocation count
    int taskReallocReload;

    //observe time
    double dt = 0.1;

    // double timeStamp;

//性能相关配置参数,大于等于2
    uint8_t observeSize;
//感知
//TODO:用sophus
    Vector2d selfPosition;
    //弧度制
    double selfRotation;
    std::vector<Vector2d> otherPosition;
    // std::vector<double> otherRotation;

    // std::vector<int8_t> taskState;

//估计
//TODO:换智能指针
    std::vector<ROBOT_ESTIMATE_STATE*> otherEstimation;
//规划
    gridPathFinder pathPlanner;
    vector<vector<Vector2d>> taskPath;
    vector<double> taskCost;
//决策
    HungarianAlgorithm allocation;
    int targetIndex;
    int taskReallocCount;
//控制
    double ctrlV = 0.3;
    double ctrlW;
    double ctrlWMax = 0.5;
    double ctrlVMax = 0.3;
    //pid
    double ctrlPidWp = 1;
    //前瞻距离
    double ctrlDis = 0.5;
    //TODO:控制限幅
//flag
    bool initFlag;

//func:
    ROBOT(GRID_MAP<Vector2d> map_, std::vector<Vector2d> taskPoints_,std::vector<Vector2d> initPosition_, uint8_t robotsNum_,uint8_t observe_size,double prob_threshold,int task_count_reload);
    //获取感知模块的输出结果
    void ROBOT_GetSenseData(const vector<Vector2d> &observe_position,const double& rotation);

    //kalman跟踪
    void ROBOT_TraceUpdata(double t);

    //初始化观测，放在循环里
    bool ROBOT_Init(double t);

    //match


    //estimate
    bool ROBOT_Estimate(void);

    //collision
    bool ROBOT_CollisionSolve(void);

    void ROBOT_Ctrl(void);

    //给外部使用的进程
    void ROBOT_Update(double t);



    

};

inline double robot_distanceL2(Vector2d x1, Vector2d x2)
{
    return (x1-x2).norm();
    // return 1.0;
}