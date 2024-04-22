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

//data
std::ofstream data_3_file("../data/data_3.csv");


double gaussian_distribution(double mean, double std_dev) 
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::normal_distribution<> dist(mean, std_dev);
    return dist(gen);
}
 


void draw_plot(void)
{
    plt::cla();
    // plt::figure_size(1200, 780);
    plt::plot(x_self, y_self,"r--");
    plt::plot(x_other, y_other,"y--");
    plt::plot(x_path, y_path,"y--");
    plt::plot(x_observed, y_observed,"g--");
    // plt::plot(x_prior, y_prior,"b--");
    plt::plot(x_match_0, y_match_0,"b");
    plt::plot(x_match_1, y_match_1,"b--");
    plt::show();
}

int main()
{


    double global_time = 0;

    vector<Vector2d> task_points;
    task_points.push_back(Vector2d(2,5));
    task_points.push_back(Vector2d(4,6));
    task_points.push_back(Vector2d(6,4));
    task_points.push_back(Vector2d(9,3));
    task_points.push_back(Vector2d(7,7));
    

    vector<Vector2d> init_points;
    init_points.push_back(Vector2d(1,2));
    init_points.push_back(Vector2d(4,3));
    init_points.push_back(Vector2d(7,2));
    init_points.push_back(Vector2d(9,6));

    init_points.push_back(Vector2d(1,1));

    //map
    int map_x = 100,map_y = 80;
    double resolution = 0.1;
    uint8_t* map_data = new uint8_t[map_x * map_y]{0};
    GRID_MAP<Vector2d> map(map_data,Vector2d(0.0,0.0),map_x,map_y,resolution);

    int robot_num = 5;

//robot
    ROBOT robot(map,task_points,init_points,robot_num,20,0.6,20);

    vector<double> v0(robot_num-1);
    vector<double> w0(robot_num-1);
    double dt = 0.1;

    //X,Y,theta,v,w
    VectorXd self_pos = (VectorXd(5) << init_points.back().x(), init_points.back().y(), M_PI_2, 0, 0).finished();
    vector<VectorXd> other_pos;
    for(int i = 0; i<robot_num-1;++i)
    {
        other_pos.push_back((VectorXd(5) << init_points.at(i).x(), init_points.at(i).y(), -M_PI_2, 0, 0).finished());
    }

    gridPathFinder other_planner;
    vector<vector<Vector2d>> other_path;

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
    for(auto& other_pos:other_pos)
    {
        Vector3d start(other_pos.x(),other_pos.y(),0);
        Vector3d end(task_points.at(1).x(),task_points.at(1).y(),0);
        other_planner.resetUsedGrids();
        other_planner.graphSearch(start,end,false);
        other_path.push_back(other_planner.getPath(true));
    }

    //data
    data_3_file << "other0_x" << ',' << "other0_y" << ',' << "other1_x" << ',' << "other1_y" << ',' << "other2_x" << ',' << "other2_y" 
                << ',' << "other3_x" << ',' << "other3_y" << ',' << "self_x" << ',' << "self_y" << '\n';


    //init
    do
    {
        global_time += dt;
        vector<Vector2d> sense_poses;
        for(auto& other_pos:other_pos)
        {
            sense_poses.push_back(Vector2d(other_pos(0),other_pos(1)));
        }
        sense_poses.push_back(Vector2d(self_pos(0),self_pos(1)));
        robot.ROBOT_GetSenseData(sense_poses,self_pos(2));
        for(int i = 0; i<robot_num-1;++i)
        {
            v0.at(i) = 0; w0.at(i) = 0;
            other_pos.at(i) = forward(other_pos.at(i),v0.at(i),w0.at(i),dt);
        }
    } while(!robot.ROBOT_Init(global_time));
    //5s
    for(int i = 0; i < 100; i++)
    {
        global_time += dt;
        vector<Vector2d> sense_poses;
        // if(i<5 || i>70)
        // {
            for(auto& other_pos:other_pos)
            {
                sense_poses.push_back(Vector2d(other_pos(0),other_pos(1)));
            }
            sense_poses.push_back(Vector2d(self_pos(0),self_pos(1)));
        // }
        // else
        // {
        //     sense_poses.push_back(Vector2d(self_pos(0),self_pos(1)));
        // }

        robot.ROBOT_GetSenseData(sense_poses,self_pos(2));
        robot.ROBOT_Update(global_time);

        for(int i = 0; i<robot_num-1;++i)
        {
            v0.at(i) = 0.3; 
            ctrl(other_path.at(i),other_pos.at(i),w0.at(i),v0.at(i));
            other_pos.at(i) = forward(other_pos.at(i),v0.at(i),w0.at(i),dt);
        }

        self_pos = forward(self_pos,robot.ctrlV,robot.ctrlW,dt);

        for(int i = 0; i<robot_num-1;++i)
        {
            data_3_file << other_pos.at(i)(0) << ',' << other_pos.at(i)(1) << ',';
        }
        data_3_file << self_pos(0) << ',' << self_pos(1) << '\n';

        // std::this_thread::sleep_for(std::chrono::milliseconds(100));   
        // draw_plot();   
    }
    data_3_file.close();
    // data_debug_file.close();
    // draw_plot();  




    return 0;
}