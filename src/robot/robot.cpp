#include "robot.hpp"

//TODO:怎么处理时间
double global_time = 0;

ROBOT::ROBOT(GRID_MAP<Vector2d> map_, std::vector<Vector2d> taskPoints_, Vector2d initPosition_, uint8_t robotsNum_,double prob_threshold,int task_count_reload)
            :map(map_),taskPoints(taskPoints_),selfPosition(initPosition_), robotsNum(robotsNum_), taskReallocReload(task_count_reload)
{
    taskReallocCount = 0;
    initFlag = false;
    Vector3d ol(map.origin.x(),map.origin.y(),0);
    Vector3d ou(map.origin.x(),map.origin.y(),1);
    pathPlanner.initGridMap(map.resolution,ol,ou,map.mapX,map.mapY,1);
    for(int i = 0; i<map.mapX;++i)
    {
        for(int j = 0; j<map.mapX;++j)
        {
            if(map.mapData[i*map.mapX + j])
            {
                pathPlanner.setObs(i,j,0);
            }
        }
    }
    taskNum = taskPoints.size();
    Matrix<double,2,2> Q;
    Q(0,0) = 0.01; Q(0,1) = 0;
    Q(1,0) = 0; Q(1,1) = 0.01;
    Matrix<double,2,2> R;
    R(0,0) = 0.0001; R(0,1) = 0;
    R(1,0) = 0; R(1,1) = 0.0001;
    VectorXd x_0 = Eigen::VectorXd::Zero(5, 1);
    for(int i = 0; i<robotsNum-1;++i)
    {
        ROBOT_ESTIMATE_STATE* other_state_ptr = new ROBOT_ESTIMATE_STATE(taskNum,prob_threshold,Q,R);
        other_state_ptr->kalman.init(x_0);
        otherEstimation.push_back(other_state_ptr);
    }
    otherPosition = vector<Vector2d>(robotsNum-1);
}

void ROBOT::ROBOT_GetSenseData()
{
}

void ROBOT::ROBOT_TraceUpdata(double t)
{
    vector<Vector2d> new_point(robotsNum-1);
    VectorXd u = VectorXd::Zero(1, 1);
    for(int i = 0;i<robotsNum-1;++i)
    {
        VectorXd x_l = otherEstimation.at(i)->kalman.predict(u,dt);
        new_point.at(i)(0) = x_l(0);
        new_point.at(i)(1) = x_l(1);
    }
    vector<vector<double>> cost;
    //行为kalman，列为实际观测
    for(int i = 0;i<robotsNum-1;++i)
    {
        vector<double> cost_raw(otherPosition.size());
        cost.push_back(cost_raw);
        for(int j = 0;j<otherPosition.size();++j)
        {
            cost.at(i).at(j) = robot_distanceL2(otherPosition.at(i),new_point.at(j));
        }
    }
    vector<int> assignment;
    allocation.Solve(cost,assignment);
    for(int i = 0;i<robotsNum-1;++i)
    {
        Vector2d observe_point;
        //未观测到
        if(assignment.at(i)==-1)
        {
            //初始化是否完成
            if(initFlag)
            {
                observe_point = otherEstimation.at(i)->ESTIMATE_EstObserve(new_point.at(i));
            }
            else
            {
                observe_point = otherEstimation.at(i)->trace.back().position;
            }
        }
        else
        {
            //用观测后验
            observe_point = VectorXd(otherPosition.at(assignment.at(i)));
        }
        //更新kalman
        VectorXd update_point(observe_point);
        Vector2d posterior_point = otherEstimation.at(i)->kalman.update(update_point);
        ROBOT_TRACE_POINT new_trace = {posterior_point,t};
        //根据trace是否足够，pushback或者push_pop
        if(otherEstimation.at(i)->trace.size()==observeSize)
        {
            otherEstimation.at(i)->trace.push_pop(new_trace);   
        }
        else
        {
            otherEstimation.at(i)->trace.push_back(new_trace);
        }
        //初始化是否完成
        if(initFlag)
        {
            otherEstimation.at(i)->ESTIMATE_JudgeTarget(taskPoints,pathPlanner);
        }
    }
}

bool ROBOT::ROBOT_Init(void)
{
    static uint8_t ready_num = 0;
    static vector<bool> ready_flag(robotsNum-1);
    //获取原始观测数据
    ROBOT_GetSenseData();
    //卡尔曼滤波匹配，加入trace
    ROBOT_TraceUpdata(global_time);
    //判断每个robot的trace观测数量是否足够
    for(uint8_t i = 0;i<robotsNum-1;++i)
    {
        if(otherEstimation.at(i)->trace.size()==observeSize && ready_flag.at(i)==false)
        {
            ready_flag.at(i) = true;
            ready_num++;
        }
    }
    //全部观测数量都足够之后
    if(ready_num==robotsNum-1)
    {
        for(uint8_t i = 0;i<robotsNum-1;++i)
        {
            //记录计算path的时刻
            otherEstimation.at(i)->pathTime = otherEstimation.at(i)->trace.back().time;
            Vector3d start(otherEstimation.at(i)->trace.back().position.x(),
                           otherEstimation.at(i)->trace.back().position.y(),
                           0);
            for(uint8_t j = 0;j<taskNum;++j)
            {
                //算tasknum次path，并存入，并算各个cost
                Vector3d end(taskPoints.at(j).x(),
                             taskPoints.at(j).y(),
                             0);
                otherEstimation.at(i)->taskCost.at(j) = pathPlanner.graphSearch(start,end,false);
                //TODO:cost为-1表示没找到路，要做处理
                otherEstimation.at(i)->taskPath.at(j) = pathPlanner.getPath(true);
            }
        }
        Vector3d start(selfPosition.x(),
                       selfPosition.y(),
                       0);
        //计算自己的path和cost
        for(uint8_t j = 0;j<taskNum;++j)
        {
            //算tasknum次path，并存入，并算各个cost
            Vector3d end(taskPoints.at(j).x(),
                            taskPoints.at(j).y(),
                            0);
            taskCost.at(j) = pathPlanner.graphSearch(start,end,false);
            taskPath.at(j) = pathPlanner.getPath(true);

        }

        //算一次task allocation，结果作为初始意图
        vector<vector<double>> cost;
        for(uint8_t i = 0;i<robotsNum-1;++i)
        {
            vector<double> robot_cost;
            for(uint8_t j = 0;j<taskNum;++j)
            {
                robot_cost.push_back(otherEstimation.at(i)->taskCost.at(j));
            }
            cost.push_back(robot_cost);
        }
        vector<double> robot_cost;
        for(uint8_t j = 0;j<taskNum;++j)
        {
            robot_cost.push_back(taskCost.at(j));
        }
        cost.push_back(robot_cost);
        vector<int> assignment;
        allocation.Solve(cost,assignment);
        for(uint8_t i = 0;i<robotsNum-1;++i)
        {
            otherEstimation.at(i)->targetIndex = assignment.at(i);
        }
        targetIndex = assignment.back();
        initFlag = true;
        return true;
    }
    else
    {
        return false;
    }
}

bool ROBOT::ROBOT_Estimate(void)
{
    return false;
}

bool ROBOT::ROBOT_CollisionSolve(void)
{
    //计算自己的path和cost
    Vector3d start(selfPosition.x(),
                   selfPosition.y(),
                   0);
    for(uint8_t j = 0;j<taskNum;++j)
    {
        //算tasknum次path，并存入，并算各个cost
        Vector3d end(taskPoints.at(j).x(),
                    taskPoints.at(j).y(),
                    0);
        taskCost.at(j) = pathPlanner.graphSearch(start,end,false);
        taskPath.at(j) = pathPlanner.getPath(true);
    }

    //算每个观测车最新点的cost
    for(uint8_t i = 0;i<robotsNum-1;++i)
    {

        Vector3d start(otherEstimation.at(i)->trace.back().position.x(),
                       otherEstimation.at(i)->trace.back().position.y(),
                       0);
        for(uint8_t j = 0;j<taskNum;++j)
        {
            //算tasknum次path，并存入，并算各个cost
            Vector3d end(taskPoints.at(j).x(),
                         taskPoints.at(j).y(),
                         0);
            otherEstimation.at(i)->taskCost.at(j) = pathPlanner.graphSearch(start,end,false);
        }
    }

    //冲突解决
    vector<bool> robot_flag(robotsNum-1,false);
    vector<bool> task_flag(robotsNum-1,false);
    bool reallocate = false;
    //最多重分配的次数
    for(uint8_t i=0;i<robotsNum-1;++i)
    {
        reallocate = false;
        for(uint8_t j = 0;j<robotsNum-1;++j)
        {
            if(otherEstimation.at(j)->targetIndex==targetIndex &&
               otherEstimation.at(j)->taskCost.at(otherEstimation.at(j)->targetIndex)<taskCost.at(targetIndex))
            {
                reallocate = true;
                robot_flag.at(j)=true;
                task_flag.at(otherEstimation.at(j)->targetIndex) = true;
                break;
            }
        }

        if(reallocate)
        {
            vector<vector<double>> cost;
            //自己是最后那个
            for(uint8_t j = 0;j<robotsNum-1;++j)
            {
                if(robot_flag.at(j))
                {
                    continue;
                }
                vector<double> robot_cost;
                for(uint8_t k = 0;k<taskNum;++k)
                {
                    if(!task_flag.at(k))
                    {
                        robot_cost.push_back(otherEstimation.at(j)->taskCost.at(k));
                    }
                }
                cost.push_back(robot_cost);
            }
            //自己
            vector<double> robot_cost;
            for(uint8_t k = 0;k<taskNum;++k)
            {
                if(!task_flag.at(k))
                {
                    robot_cost.push_back(taskCost.at(k));
                }
            }
            cost.push_back(robot_cost);
            vector<int> assignment;
            allocation.Solve(cost,assignment);
            targetIndex = assignment.back();
        }
        //冲突已解决，不清空意图，设置一个计数器，一段时间后再检测冲突
        else
        {
            for(uint8_t j = 0;j<robotsNum-1;++j)
            {
                taskReallocCount = taskReallocReload;
            }
            break;
        }
    }
    return false;
}

void ROBOT::ROBOT_Ctrl(void)
{
}

void ROBOT::ROBOT_Process(void)
{
    //获取原始观测数据
    ROBOT_GetSenseData();
    //卡尔曼滤波匹配，加入trace
    ROBOT_TraceUpdata(global_time);

    if(taskReallocCount<=0)
    {
        for(uint8_t i = 0;i<robotsNum-1;++i)
        {
            if(otherEstimation.at(i)->targetIndex==targetIndex)
            {
                Vector3d start(otherEstimation.at(i)->trace.back().position.x(),
                            otherEstimation.at(i)->trace.back().position.y(),
                            0);
                Vector3d end(taskPoints.at(targetIndex).x(),
                            taskPoints.at(targetIndex).y(),
                            0);
                double other_cost = pathPlanner.graphSearch(start,end,false);
                
                Vector3d self_start(selfPosition.x(),
                                selfPosition.y(),
                                0);
                double self_cost = pathPlanner.graphSearch(self_start,end,false);
                if(other_cost<self_cost)
                {
                    ROBOT_CollisionSolve();
                    break;
                }
            }
        }
    }
    else
    {
        taskReallocCount--;
    }

    
    //4.执行自己的命令
}

inline void ROBOT_ESTIMATE_STATE::ESTIMATE_JudgeTarget(const vector<Vector2d> &task_points,gridPathFinder &path_planner)
{
    //更新距离
    distance += robot_distanceL2(trace.at(trace.size()-2).position,trace.back().position);
    //更新estPoint和taskProb
    ESTIMATE_estUpdate();
    ESTIMATE_probUpdate();

    //TODO:t0距离现在时刻太远也要重规划
    //概率最大的小于阈值，重规划
    if(taskProb.at(targetIndex)<probThreshold)
    {
        //用最老的点重新规划
        Vector3d start(trace.at(0).position.x(),
                        trace.at(0).position.y(),
                        0);
        for(uint8_t i = 0;i<taskPath.size();++i)
        {
            Vector3d end(task_points.at(i).x(),
                        task_points.at(i).y(),
                        0);
            taskCost.at(i) = path_planner.graphSearch(start,end,false);
            taskPath.at(i) = path_planner.getPath(true);
        }
        pathTime = trace.at(0).time;
        distance = 0;
        for(uint8_t i = 1;i<trace.size();++i)
        {
            distance += robot_distanceL2(trace.at(i).position,trace.at(i-1).position);
        }
        //更新estPoint和taskProb
        ESTIMATE_estUpdate();
        ESTIMATE_probUpdate();
    }
 
}

//TODO:用taskPathLastIndex来记录上次的index，这样可以减少计算量
inline void ROBOT_ESTIMATE_STATE::ESTIMATE_estUpdate(void)
{
    for(uint8_t i = 0;i<taskPath.size();++i)
    {
        estPoint.at(i) = ESTIMATE_findEst(i,distance);
    }
}

//TODO:先用欧氏距离，后面再换成概率
//TODO:概率用贝叶斯，注意重规划的时候要清零
inline void ROBOT_ESTIMATE_STATE::ESTIMATE_probUpdate(void)
{
    vector<double> match_dis(estPoint.size());
    double sum_e_dis = 0;
    for(uint8_t i = 0;i<match_dis.size();++i)
    {
        match_dis.at(i) = exp(-robot_distanceL2(trace.back().position,estPoint.at(i)));
        sum_e_dis += match_dis.at(i);
    }
    for(uint8_t i = 0;i<match_dis.size();++i)
    {
        taskProb.at(i) = match_dis.at(i)/sum_e_dis;
    }
    auto max_it = std::max_element(taskProb.begin(), taskProb.end());
    targetIndex = std::distance(taskProb.begin(), max_it);
}

inline Vector2d ROBOT_ESTIMATE_STATE::ESTIMATE_EstObserve(Vector2d new_point)
{
    double trace_dis = distance + robot_distanceL2(new_point,trace.back().position);
    return ESTIMATE_findEst(targetIndex,trace_dis);
}

inline Vector2d ROBOT_ESTIMATE_STATE::ESTIMATE_findEst(int path_idx, double trace_dis)
{
    double path_dis = 0;
    for(int i = 0;i<taskPath.at(path_idx).size();++i)
    {
        if(i!=0)
        {
            path_dis += robot_distanceL2(taskPath.at(path_idx).at(i),taskPath.at(path_idx).at(i-1));
        }
        if(trace_dis<path_dis)
        {
            if(i==0)
            {
                return taskPath.at(path_idx).at(0);
            }
            else
            {
                double proportion = (path_dis-trace_dis)/robot_distanceL2(taskPath.at(path_idx).at(i),taskPath.at(path_idx).at(i-1));
                Vector2d path_match = proportion*taskPath.at(path_idx).at(i-1) + (1-proportion)*taskPath.at(path_idx).at(i);
                return path_match;
            }
        }
    }
    //path搜索完了，返回最后一个
    return taskPath.at(path_idx).back();
}

inline double robot_distanceL2(Vector2d x1, Vector2d x2)
{
    return (x1-x2).norm();
}

// inline void ROBOT_ESTIMATE_STATE::ESTIMATE_matchPathTrace(void)
// {
//     //匹配path中的trace点
//     int trace_total = trace.size();
//     for(uint8_t i = 0;i<taskPath.size();++i)
//     {
//         double path_dis = 0;
//         double trace_dis = distance;
//         int trace_count = 0;
//         //
//         for(int j = 0;j<taskPath.at(i).size();++j)
//         {
//             //
//             if(j!=0)
//             {
//                 path_dis += robot_distanceL2(taskPath.at(i).at(j),taskPath.at(i).at(j-1));
//             }
//             //
//             while(trace_dis<path_dis)
//             {
//                 if(j==0)
//                 {
//                     estimateTrace.at(i).push_back(taskPath.at(i).at(0));
//                 }
//                 else
//                 {
//                     double proportion = (path_dis-trace_dis)/robot_distanceL2(taskPath.at(i).at(j),taskPath.at(i).at(j-1));
//                     Vector2d path_match = proportion*taskPath.at(i).at(j-1) + (1-proportion)*taskPath.at(i).at(j);
//                     estimateTrace.at(i).push_back(path_match);
//                 }
//                 trace_count++;
//                 trace_dis += robot_distanceL2(trace.at(trace_count-1).position,trace.at(trace_count).position);
//                 if(trace_count==trace_total)
//                 {
//                     break;
//                 }
//             }
//             if(trace_count==trace_total)
//             {
//                 taskPathLastIndex.at(i) = j;
//                 break;
//             }
//         }
//         while(trace_count<trace_total)
//         {
//             estimateTrace.at(i).push_back(taskPath.at(i).back());
//             trace_count ++;
//         }
//     }

// }

// //TODO:先拿欧氏距离算，后面再换成跟概率有关的
// inline void ROBOT_ESTIMATE_STATE::ESTIMATE_calculateProb(void)
// {
//     double sum_dis = 0;
//     for(uint8_t i=0;i<taskPath.size();++i)
//     {
//         matchDis.at(i) = 0;
//         for(int j=0;j<trace.size();++j)
//         {
//             matchDis.at(i) += robot_distanceL2(trace.at(j).position,estimateTrace.at(i).at(j));
//         }
//         sum_dis += exp(matchDis.at(i));
//     }
//     for(uint8_t i=0;i<taskPath.size();++i)
//     {
//         taskProb.at(i) = exp(matchDis.at(i))/sum_dis;
//     }
//     auto max_it = std::max_element(taskProb.begin(), taskProb.end());
//     targetIndex = std::distance(taskProb.begin(), max_it);
// }


// inline void ROBOT_ESTIMATE_STATE::ESTIMATE_ProbUpdate(ROBOT_TRACE_POINT old_trace)
// {
//     double sum_dis = 0;
//     for(uint8_t i = 0;i<taskPath.size();++i)
//     {
//         //离estimateTrace最后一个点最近的后面一个path点，到他的距离
//         double path_dis = robot_distanceL2(taskPath.at(i).at(taskPathLastIndex.at(i)),estimateTrace.at(i).back());
//         double trace_dis = robot_distanceL2(trace.back().position,trace.at(trace.size()-2).position);
//         bool find_flag = false;
        
//         //对每个task的path-trace更新距离之和
//         matchDis.at(i) -= robot_distanceL2(old_trace.position,estimateTrace.at(i).at(0));

//         for(int j = taskPathLastIndex.at(i);j<taskPath.at(i).size();++j)
//         {
//             if(j!=taskPathLastIndex.at(i))
//             {
//                 path_dis += robot_distanceL2(taskPath.at(i).at(j),taskPath.at(i).at(j-1));
//             }
//             if(trace_dis<path_dis)
//             {
//                 if(j==0)
//                 {
//                     estimateTrace.at(i).push_pop(taskPath.at(i).at(0));
//                 }
//                 else if(j==taskPathLastIndex.at(i))
//                 {
//                     double proportion = (path_dis-trace_dis)/robot_distanceL2(taskPath.at(i).at(j),estimateTrace.at(i).back());
//                     Vector2d path_match = proportion*estimateTrace.at(i).back() + (1-proportion)*taskPath.at(i).at(j);
//                     estimateTrace.at(i).push_pop(path_match);
//                 }
//                 else
//                 {
//                     double proportion = (path_dis-trace_dis)/robot_distanceL2(taskPath.at(i).at(j),taskPath.at(i).at(j-1));
//                     Vector2d path_match = proportion*taskPath.at(i).at(j-1) + (1-proportion)*taskPath.at(i).at(j);
//                     estimateTrace.at(i).push_pop(path_match);
//                 }
//                 taskPathLastIndex.at(i) = j;
//                 find_flag = true;
//                 break;
//             }
//         }
//         if(!find_flag)
//         {
//             estimateTrace.at(i).push_pop(taskPath.at(i).back());
//         }
//         //对每个task的path-trace更新距离之和
//         matchDis.at(i) += robot_distanceL2(trace.back().position,estimateTrace.at(i).back());
//         sum_dis += exp(matchDis.at(i));
//     }
//     for(uint8_t i=0;i<taskPath.size();++i)
//     {
//         taskProb.at(i) = exp(matchDis.at(i))/sum_dis;
//     }
// }

