#include "robot.hpp"
#include "matplotlibcpp.h"
#include <iostream>
#include <random>
namespace plt = matplotlibcpp;

double gaussian_distribution(double mean, double std_dev) 
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::normal_distribution<> dist(mean, std_dev);
    return dist(gen);
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

void ctrl(const vector<Vector2d>& task_path,const VectorXd& state ,double& ctrlW)
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
    }
    else
    {
        track_point = task_path.at(min_idx+1);
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
    ctrlW = 1 * angle_diff;
    double ctrlWMax = 0.5;
    ctrlW = ctrlW>ctrlWMax?ctrlWMax : ctrlW<-ctrlWMax?-ctrlWMax:ctrlW;
}

int main()
{
    double global_time = 0;

    vector<Vector2d> task_points;
    task_points.push_back(Vector2d(1,4));
    task_points.push_back(Vector2d(6,5));

    vector<Vector2d> init_points;
    init_points.push_back(Vector2d(3,1));
    init_points.push_back(Vector2d(5,8));

    //map
    int map_x = 8,map_y = 10;
    double resolution = 1.0;
    uint8_t* map_data = new uint8_t[map_x * map_y]{0};
    GRID_MAP<Vector2d> map(map_data,Vector2d(0.0,0.0),map_x,map_y,resolution);

    int robot_num = 2;

    ROBOT robot(map,task_points,init_points,robot_num,3,0.5,3);

    double v0 = 0.0;
    double w0 = 0.0;
    double dt = 0.1;

    VectorXd self_pos = (VectorXd(5) << 3, 1, 0, 0, 0).finished();

    //X,Y,theta,v,w
    VectorXd other_pos = (VectorXd(5) << 5, 8, 0, 0, 0).finished();

    gridPathFinder other_planner;
    vector<Vector2d> other_path;

    Vector3d ol(map.origin.x(),map.origin.y(),0);
    Vector3d ou(map.origin.x(),map.origin.y(),1);
    other_planner.initGridMap(map.resolution,ol,ou,map.mapX,map.mapY,1);
    for(int i = 0; i<map.mapX;++i)
    {
        for(int j = 0; j<map.mapX;++j)
        {
            if(map.mapData[i*map.mapX + j])
            {
                other_planner.setObs(i,j,0);
            }
        }
    }

    Vector3d start(other_pos.x(),other_pos.y(),0);
    Vector3d end(task_points.at(1).x(),task_points.at(1).y(),0);
    other_planner.graphSearch(start,end,false);
    other_path = other_planner.getPath(true);

    //plot
    std::vector<double> x_self,y_self,x_other,y_other;

    while(!robot.ROBOT_Init(global_time))
    {
        global_time += dt;
        vector<Vector2d> sense_poses = {Vector2d(other_pos(0),other_pos(1)),Vector2d(self_pos(0),self_pos(1))};
        robot.ROBOT_GetSenseData(sense_poses,self_pos(2));
        v0 = 0; w0 = 0;
        other_pos = forward(other_pos,v0,w0,dt);
    }
    //5s
    for(int i = 0; i < 50; i++)
    {
        global_time += dt;
        vector<Vector2d> sense_poses = {Vector2d(other_pos(0),other_pos(1)),Vector2d(self_pos(0),self_pos(1))};
        robot.ROBOT_GetSenseData(sense_poses,self_pos(2));
       
        ctrl(other_path,other_pos,w0);
        v0 = 0.3;
        other_pos = forward(other_pos,v0,w0,dt);

        robot.ROBOT_Update(global_time);
        self_pos = forward(self_pos,robot.ctrlV,robot.ctrlW,dt);

        x_self.push_back(self_pos(0));
        y_self.push_back(self_pos(1));
        x_other.push_back(other_pos(0));
        y_other.push_back(other_pos(1));

        
    }

    plt::figure_size(1200, 780);

    plt::plot(x_self, y_self,"r--");
    plt::plot(x_other, y_other);

    plt::legend();
    plt::show();

    return 0;
}