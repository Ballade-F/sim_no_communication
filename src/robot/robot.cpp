#include "robot.hpp"

//TODO:怎么处理时间
double global_time = 0;

ROBOT::ROBOT(GRID_MAP<Vector2d> map_, std::vector<Vector2d> taskPoints_, Vector2d initPosition_, uint8_t robotsNum_):map(map_),taskPoints(taskPoints_),selfPosition(initPosition_), robotsNum(robotsNum_)
{
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
        ROBOT_ESTIMATE_STATE* other_state_ptr = new ROBOT_ESTIMATE_STATE(taskNum,Q,R);
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
    // MatrixXd cost = MatrixXd::Zero(robotsNum-1, robotsNum-1);
    //行为kalman，列为实际观测
    for(int i = 0;i<robotsNum-1;++i)
    {
        vector<double> cost_raw(otherPosition.size());
        cost.push_back(cost_raw);
        for(int j = 0;j<otherPosition.size();++j)
        {
            cost.at(i).at(j) = ROBOT_distanceL2(otherPosition.at(i),new_point.at(j));
        }
    }
    // for(int i = 0;i<otherPosition.size();++i)
    // {
    //     vector<double> cost_raw(robotsNum-1);
    //     cost.push_back(cost_raw);
    //     for(int j = 0;j<robotsNum-1;++j)
    //     {
    //         cost.at(i).push_back(ROBOT_distanceL2(otherPosition.at(i),new_point.at(j)));
    //     }
    // }
    vector<int> assignment;
    allocation.Solve(cost,assignment);
    for(int i = 0;i<robotsNum-1;++i)
    {
        if(assignment.at(i)==-1)
        {
            //TODO:处理未观测到的
        }
        else
        {
            //用观测后验
            // VectorXd z = Eigen::VectorXd::Zero(2, 1);
            VectorXd observe_point(otherPosition.at(assignment.at(i)));
            Vector2d posterior_point = otherEstimation.at(i)->kalman.update(observe_point);
            ROBOT_TRACE_POINT new_trace = {posterior_point,global_time};

            otherEstimation.at(i)->distance += ROBOT_distanceL2(posterior_point,otherEstimation.at(i)->trace.back().position);

            if(otherEstimation.at(i)->trace.size()==observeSize)
            {
                otherEstimation.at(i)->trace.push_pop(new_trace);
            }
            else
            {
                otherEstimation.at(i)->trace.push_back(new_trace);
            }
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
            otherEstimation.at(i)->taskPath.clear();
            otherEstimation.at(i)->taskCost.clear();
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

        //TODO:算一次task allocation

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
    return false;
}

void ROBOT::ROBOT_Ctrl(void)
{
}

void ROBOT::ROBOT_Process(void)
{
    //1.获取sensedata，kalman跟踪，更新trace

    //for每一个other
        //2.trace中的t全都新于path的t，则

            //2.1.如果是观测到的，通过trace和path计算各task的概率

            //2.2.若最大概率都小于阈值，则用trace中最老的重算path，再算各个task的概率,pathTime,distance

            //2.3.将概率最大的视为目标

    //3.冲突解决
    //while（有其他人的target与自己一致且cost<自己）
        //重分配

    //4.执行自己的命令
}

inline double ROBOT::ROBOT_distanceL2(Vector2d x1, Vector2d x2)
{
    return (x1-x2).norm();
}

void ROBOT_ESTIMATE_STATE::ESTIMATE_CalculateProb(void)
{
    for(int i=0;i<taskPath.size();++i)
    {
        
    }
}
