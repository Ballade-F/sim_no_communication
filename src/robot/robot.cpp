#include "robot.hpp"
#include <fstream>
//TODO:怎么处理时间


//debug
std::vector<double> x_prior,y_prior;
std::ofstream data_debug_file("../data/data_2_debug.csv");

//data
std::ofstream data_file("../data/data_2.csv");

ROBOT::ROBOT(GRID_MAP<Vector2d> map_, std::vector<Vector2d> taskPoints_, std::vector<Vector2d> initPosition_, uint8_t robotsNum_,uint8_t observe_size,double prob_threshold,int task_count_reload)
            :map(map_),taskPoints(taskPoints_), robotsNum(robotsNum_), observeSize(observe_size) ,taskReallocReload(task_count_reload),replanCount(task_count_reload)
{
    taskNum = taskPoints.size();
    selfPosition = initPosition_.back();
    taskReallocCount = taskReallocReload;

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
    
    Matrix<double,2,2> Q;
    Q(0,0) = 0.02; Q(0,1) = 0;
    Q(1,0) = 0; Q(1,1) = 0.02;
    Matrix<double,2,2> R;
    R(0,0) = 0.0001; R(0,1) = 0;
    R(1,0) = 0; R(1,1) = 0.0001;
    VectorXd x_0 = Eigen::VectorXd::Zero(5, 1);
    for(int i = 0; i<robotsNum-1;++i)
    {
        ROBOT_ESTIMATE_STATE* other_state_ptr = new ROBOT_ESTIMATE_STATE(taskNum,prob_threshold,initPosition_.at(i),Q,R);
        x_0(0) = initPosition_.at(i).x();
        x_0(1) = initPosition_.at(i).y();
        //debug
        x_0(2) = -M_PI_2;
        x_0(3) = 0.0;
        other_state_ptr->kalman.init(x_0);
        otherEstimation.push_back(other_state_ptr);
    }
    // otherPosition = vector<Vector2d>(robotsNum-1);
    taskPath = vector<vector<Vector2d>>(taskNum);
    taskCost = vector<double>(taskNum);
    targetIndex = -1;
    ctrlW = 0;
    initFlag = false;
}


bool ROBOT::ROBOT_Init(double t)
{
    static uint8_t ready_num = 0;
    static vector<bool> ready_flag(robotsNum-1);
    //获取原始观测数据
    // ROBOT_GetSenseData();
    //卡尔曼滤波匹配，加入trace
    ROBOT_TraceUpdata(t);
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
            // //cout trace
            // for(uint8_t j = 0;j<otherEstimation.at(i)->trace.size();++j)
            // {
            //     cout<<otherEstimation.at(i)->trace.at(j).position.x()<<" "<<otherEstimation.at(i)->trace.at(j).position.y()<<endl;
            // }
            // cout << " "<<endl;
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
                pathPlanner.resetUsedGrids();
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
            Vector3d end(taskPoints.at(j).x(),taskPoints.at(j).y(),0);
            pathPlanner.resetUsedGrids();
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

        //debug
        //cout cost
        for(uint8_t i = 0;i<robotsNum;++i)
        {
            for(uint8_t j = 0;j<taskNum;++j)
            {
                cout<<cost.at(i).at(j)<<" ";
            }
            cout<<endl;
        }
        

        vector<int> assignment;
        allocation.Solve(cost,assignment);
        for(uint8_t i = 0;i<robotsNum-1;++i)
        {
            otherEstimation.at(i)->targetIndex = assignment.at(i);
        }
        targetIndex = assignment.back();
        initFlag = true;
        taskReallocCount = taskReallocReload;
        return true;
    }
    else
    {
        return false;
    }
}

void ROBOT::ROBOT_Update(double t)
{
    //获取原始观测数据
    // ROBOT_GetSenseData();
    //卡尔曼滤波匹配，加入trace
    ROBOT_TraceUpdata(t);

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
                pathPlanner.resetUsedGrids();
                double other_cost = pathPlanner.graphSearch(start,end,false);
                
                Vector3d self_start(selfPosition.x(),
                                selfPosition.y(),
                                0);
                pathPlanner.resetUsedGrids();
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
    ROBOT_Ctrl();
}


void ROBOT::ROBOT_GetSenseData(const vector<Vector2d> &observe_position, const double& rotation)
{
    selfPosition = observe_position.back();
    otherPosition.clear();
    for(int i = 0;i<observe_position.size()-1;++i)
    {
        otherPosition.push_back(observe_position.at(i));
    }
    selfRotation = rotation;
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
                if(otherEstimation.at(i)->findFlag)
                {
                    otherEstimation.at(i)->lastEst = otherEstimation.at(i)->kalman.x_k;
                    otherEstimation.at(i)->findFlag = false;
                }
                observe_point = otherEstimation.at(i)->ESTIMATE_EstObserve(dt);
            }
            else
            {
                observe_point = otherEstimation.at(i)->trace.back().position;
            }
        }
        else
        {
            //用观测后验
            //TODO:处理再次观测到
            otherEstimation.at(i)->findFlag = true;
            observe_point = VectorXd(otherPosition.at(assignment.at(i)));
        }
        //更新kalman
        VectorXd update_point(observe_point);
        VectorXd posterior_point = otherEstimation.at(i)->kalman.update(update_point);
        ROBOT_TRACE_POINT new_trace = {Vector2d(posterior_point(0),posterior_point(1)),t};

        //debug
        //cout kalman
        x_prior.push_back(new_point.at(i).x());
        y_prior.push_back(new_point.at(i).y());
        // cout <<i<<": "<< "prior: "<< new_point.at(i).x()<<" "<<new_point.at(i).y()<<endl;
        // cout << "   observe: "<< observe_point.x()<<" "<<observe_point.y()<<endl;
        // cout << "   posterior: "<< posterior_point(0)<<" "<<posterior_point(1)<<" "<<posterior_point(2)<<" "<<posterior_point(3)<<" "<<posterior_point(4)<<endl;

        //data
        data_file<<new_point.at(i).x()<<","<<new_point.at(i).y()<<","<<observe_point.x()<<","<<observe_point.y()<<",";
        data_file<<posterior_point(0)<<","<<posterior_point(1)<<","<<posterior_point(2)<<","<<posterior_point(3)<<","<<posterior_point(4)<<"\n";

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
            bool replan_flag = false;
            if(replanCount>=0)
            {
                replanCount--;
            }
            else
            {
                replan_flag = true;
            }
            otherEstimation.at(i)->ESTIMATE_JudgeTarget(taskPoints,pathPlanner,replan_flag);

            //debug
            //cout taskProb
            for(uint8_t j = 0;j<taskNum;++j)
            {
                cout<<otherEstimation.at(i)->taskProb.at(j)<<" ";
            }
            cout<<endl;
        }
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
        pathPlanner.resetUsedGrids();
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
            pathPlanner.resetUsedGrids();
            otherEstimation.at(i)->taskCost.at(j) = pathPlanner.graphSearch(start,end,false);
        }
    }

    //冲突解决
    vector<bool> robot_flag(robotsNum-1,false);
    vector<bool> task_flag(taskNum,false);
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
            //标记任务的实际序号
            vector<int8_t> task_idx;
            vector<double> robot_cost;
            for(uint8_t k = 0;k<taskNum;++k)
            {
                if(!task_flag.at(k))
                {
                    robot_cost.push_back(taskCost.at(k));
                    task_idx.push_back(k);
                }
            }
            cost.push_back(robot_cost);
            
            vector<int> assignment;
            allocation.Solve(cost,assignment);
            targetIndex = task_idx.at(assignment.back());
        }
        //冲突已解决，不清空意图，设置一个计数器，一段时间后再检测冲突
        else
        {
            break;
        }
    }
    taskReallocCount = taskReallocReload;
    return false;
}

void ROBOT::ROBOT_Ctrl(void)
{
    //TODO:轨迹生成
    //用最近点后面前瞻距离的一个点作为跟踪点
    Vector2d track_point = taskPath.at(targetIndex).at(0);
    double min_dis = robot_distanceL2(track_point,selfPosition);
    int min_idx = 0;
    for(uint8_t i = 1;i<taskPath.at(targetIndex).size();++i)
    {
        double dis = robot_distanceL2(taskPath.at(targetIndex).at(i),selfPosition);
        if(dis<min_dis)
        {
            min_dis = dis;
            min_idx = i;
        }
    }

    if(min_idx==taskPath.at(targetIndex).size()-1)
    {
        track_point = taskPath.at(targetIndex).back();
        ctrlV = min_dis;
    }
    else
    {
        //向后搜索前瞻距离的点作为目标点
        double dis = 0;
        bool find_flag = false;
        for(uint8_t i = min_idx;i<taskPath.at(targetIndex).size()-1;++i)
        {
            dis += robot_distanceL2(taskPath.at(targetIndex).at(i),taskPath.at(targetIndex).at(i+1));
            if(dis>ctrlDis)
            {
                track_point = taskPath.at(targetIndex).at(i);
                find_flag = true;
                break;
            }
        }
        //快到终点了，用终点，减速
        if(!find_flag)
        {
            track_point = taskPath.at(targetIndex).back();
            ctrlV = robot_distanceL2(track_point,selfPosition);
        }
    }

    //控制角速度
    double target_angle = atan2(track_point.y()-selfPosition.y(),track_point.x()-selfPosition.x());
    double angle_diff = target_angle - selfRotation;
    if(angle_diff>M_PI)
    {
        angle_diff -= 2*M_PI;
    }
    else if(angle_diff<-M_PI)
    {
        angle_diff += 2*M_PI;
    }
    ctrlW = ctrlPidWp * angle_diff;
    ctrlW = ctrlW>ctrlWMax?ctrlWMax : ctrlW<-ctrlWMax?-ctrlWMax:ctrlW;
    ctrlV = ctrlV>ctrlVMax?ctrlVMax:ctrlV<-ctrlVMax?-ctrlVMax:ctrlV;
}


inline void ROBOT_ESTIMATE_STATE::ESTIMATE_JudgeTarget(const vector<Vector2d> &task_points,gridPathFinder &path_planner, bool replan_flag)
{

    //debug
    static int count = 0;
    //更新距离
    distance += robot_distanceL2(trace.at(trace.size()-2).position,trace.back().position);
    //更新estPoint和taskProb
    ESTIMATE_estUpdate();
    ESTIMATE_probUpdate(replan_flag);

    //TODO:t0距离现在时刻太远也要重规划
    //概率最大的小于阈值，重规划
    if(replan_flag && taskProb.at(targetIndex)<probThreshold)
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
            path_planner.resetUsedGrids();
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
        ESTIMATE_probUpdate(replan_flag);
        //debug
        //cout re-plan
        cout<<"re-plan"<<endl;
        // count++;
        // if(count==2)
        // {
        //     for(uint8_t i = 0;i<taskPath.at(0).size();++i)
        //     {
        //         for(uint8_t j = 0;j<taskPath.size();++j)
        //         {
        //             data_debug_file<<taskPath.at(j).at(i).x()<<","<<taskPath.at(j).at(i).y()<<",";
        //         }
        //         data_debug_file<<'\n';
        //     }
        // }
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
inline void ROBOT_ESTIMATE_STATE::ESTIMATE_probUpdate(bool replan_flag)
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
    if(findFlag && replan_flag)
    {
        auto max_it = std::max_element(taskProb.begin(), taskProb.end());
        targetIndex = std::distance(taskProb.begin(), max_it);
    }

}

inline Vector2d ROBOT_ESTIMATE_STATE::ESTIMATE_EstObserve(double dt)
{

    double est_v = 0.3;
    double est_w;

    ctrl(taskPath.at(targetIndex),lastEst,est_w,est_v,0.6);
    lastEst = forward(lastEst,est_v,est_w,dt);
    Vector2d est_point_2d(lastEst(0),lastEst(1));
    return est_point_2d;

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



VectorXd forward(VectorXd x_, double v_ , double w_, double t_)
{
    VectorXd x = x_;
    x(0,0) += t_ * x_(3,0) * cos(x_(2,0));
	x(1,0) += t_ * x_(3,0) * sin(x_(2,0));
	x(2,0) += t_ * x_(4,0);
    x(3,0) = v_;
    x(4,0) = w_;
    return x;
}

void ctrl(const vector<Vector2d>& task_path,const VectorXd& state ,double& ctrlW,double& ctrlV,const double ctrlDis )
{
    //TODO:轨迹生成，找到path最近点，若小于前瞻距离，则向前寻找前瞻点，否则用最近点
    //用最近点后面的一个点作为跟踪点
    Vector2d track_point = task_path.at(0);
    Vector2d position(state(0),state(1));
    double rotation = state(2);
    double min_dis = robot_distanceL2(track_point,position);
    int min_idx = 0;
    for(uint8_t i = 1;i<task_path.size();++i)
    {
        double dis = robot_distanceL2(task_path.at(i),position);
        if(dis<min_dis)
        {
            min_dis = dis;
            min_idx = i;
        }
    }
    if(min_idx==task_path.size()-1)
    {
        track_point = task_path.back();
        ctrlV = min_dis;
    }
    else
    {
        //向后搜索前瞻距离的点作为目标点
        double dis = 0;
        bool find_flag = false;
        for(uint8_t i = min_idx;i<task_path.size()-1;++i)
        {
            dis += robot_distanceL2(task_path.at(i),task_path.at(i+1));
            if(dis>ctrlDis)
            {
                track_point = task_path.at(i);
                find_flag = true;
                break;
            }
        }
        //快到终点了，用终点，减速
        if(!find_flag)
        {
            track_point = task_path.back();
            ctrlV = robot_distanceL2(track_point,position);
        }
    }

    //控制角速度
    double target_angle = atan2(track_point.y()-position.y(),track_point.x()-position.x());
    double angle_diff = target_angle - rotation;
    if(angle_diff>M_PI)
    {
        angle_diff -= 2*M_PI;
    }
    else if(angle_diff<-M_PI)
    {
        angle_diff += 2*M_PI;
    }
    ctrlW = 2 * angle_diff;
    double ctrlWMax = 0.5;
    ctrlW = ctrlW>ctrlWMax?ctrlWMax : ctrlW<-ctrlWMax?-ctrlWMax:ctrlW;
    double ctrlVMax = 0.3;
    ctrlV = ctrlV>ctrlVMax?ctrlVMax:ctrlV<-ctrlVMax?-ctrlVMax:ctrlV;
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

