#include "robot.hpp"
#include "matplotlibcpp.h"
#include <iostream>
#include <random>
#include <thread>
#include <chrono>


namespace plt = matplotlibcpp;

//plot
std::vector<double> x_self,y_self,x_other,y_other,x_path,y_path;
std::vector<double> x_observed,y_observed;
std::vector<double> x_match_0,y_match_0;
std::vector<double> x_match_1,y_match_1;


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

void ctrl(const vector<Vector2d>& task_path,const VectorXd& state ,double& ctrlW,double& ctrlV,const double ctrlDis = 0.6)
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

void draw_plot(void)
{
    plt::cla();
    // plt::figure_size(1200, 780);
    plt::plot(x_self, y_self,"r--");
    plt::plot(x_other, y_other,"y--");
    plt::plot(x_path, y_path,"y--");
    plt::plot(x_observed, y_observed,"g--");
    plt::plot(x_prior, y_prior,"b--");
    plt::plot(x_match_0, y_match_0,"b--");
    plt::plot(x_match_1, y_match_1,"b--");
    plt::show();
}

int main()
{
    //data
    data_file << "prior_x" << ',' << "prior_y" << ',' << "observe_x" << ',' << "observe_y" << ','
             << "post_x" << ',' << "post_y" << ',' << "post_theta" << ',' << "post_v" << ',' << "post_w" << '\n';

    double global_time = 0;

    vector<Vector2d> task_points;
    task_points.push_back(Vector2d(4.5,5.5));
    task_points.push_back(Vector2d(6.5,5.5));

    vector<Vector2d> init_points;
    init_points.push_back(Vector2d(5.5,8.5));
    init_points.push_back(Vector2d(3.5,1.5));

    //map
    int map_x = 80,map_y = 100;
    double resolution = 0.1;
    uint8_t* map_data = new uint8_t[map_x * map_y]{0};
    GRID_MAP<Vector2d> map(map_data,Vector2d(0.0,0.0),map_x,map_y,resolution);

    int robot_num = 2;

//robot
    ROBOT robot(map,task_points,init_points,robot_num,20,0.6,20);

    double v0 = 0.0;
    double w0 = 0.0;
    double dt = 0.1;

    //X,Y,theta,v,w
    VectorXd self_pos = (VectorXd(5) << init_points.back().x(), init_points.back().y(), M_PI_2, 0, 0).finished();
    VectorXd other_pos = (VectorXd(5) << init_points.at(0).x(), init_points.at(0).y(), -M_PI_2, 0, 0).finished();

    gridPathFinder other_planner;
    vector<Vector2d> other_path;

    //init other planner
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
    //get path
    Vector3d start(other_pos.x(),other_pos.y(),0);
    Vector3d end(task_points.at(0).x(),task_points.at(0).y(),0);
    other_planner.graphSearch(start,end,false);
    other_path = other_planner.getPath(true);


    for(auto& point:other_path)
    {
        x_path.push_back(point.x());
        y_path.push_back(point.y());
    }

    //init
    do
    {
        global_time += dt;
        vector<Vector2d> sense_poses = {Vector2d(other_pos(0),other_pos(1)),Vector2d(self_pos(0),self_pos(1))};
        robot.ROBOT_GetSenseData(sense_poses,self_pos(2));
        v0 = 0; w0 = 0;
        other_pos = forward(other_pos,v0,w0,dt);
    } while(!robot.ROBOT_Init(global_time));
    //5s
    for(int i = 0; i < 200; i++)
    {
        global_time += dt;
        vector<Vector2d> sense_poses = {Vector2d(other_pos(0),other_pos(1)),Vector2d(self_pos(0),self_pos(1))};
        robot.ROBOT_GetSenseData(sense_poses,self_pos(2));
        robot.ROBOT_Update(global_time);

        v0 = 0.3;
        ctrl(other_path,other_pos,w0,v0);
        other_pos = forward(other_pos,v0,w0,dt);
        //debug
        //cout w0
        cout<<"w:"<<w0<<endl;
        self_pos = forward(self_pos,robot.ctrlV,robot.ctrlW,dt);

        x_self.push_back(self_pos(0));
        y_self.push_back(self_pos(1));
        x_other.push_back(other_pos(0));
        y_other.push_back(other_pos(1));
        x_observed.push_back(robot.otherEstimation.at(0)->trace.back().position.x());
        y_observed.push_back(robot.otherEstimation.at(0)->trace.back().position.y());
        x_match_0.push_back(robot.otherEstimation.at(0)->estPoint.at(0).x());
        y_match_0.push_back(robot.otherEstimation.at(0)->estPoint.at(0).y());
        x_match_1.push_back(robot.otherEstimation.at(0)->estPoint.at(1).x());
        y_match_1.push_back(robot.otherEstimation.at(0)->estPoint.at(1).y()); 
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));   
        // draw_plot();   
    }
    data_file.close();
    draw_plot();  




    return 0;
}