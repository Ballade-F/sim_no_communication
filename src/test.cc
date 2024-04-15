#include <iostream>
#include "robot.hpp"

int main()
{
    vector<Vector2d> task_points;
    task_points.push_back(Vector2d(1,9));
    task_points.push_back(Vector2d(2,9));
    task_points.push_back(Vector2d(3,9));
    task_points.push_back(Vector2d(4,9));
    task_points.push_back(Vector2d(5,9));

    vector<Vector2d> init_points;
    init_points.push_back(Vector2d(1,1));
    init_points.push_back(Vector2d(2,2));
    init_points.push_back(Vector2d(3,1));
    init_points.push_back(Vector2d(4,2));
    init_points.push_back(Vector2d(5,1));

    //map
    int map_x = 10,map_y = 10;
    double resolution = 1.0;
    uint8_t* map_data = new uint8_t[map_x * map_y]{0};
    GRID_MAP<Vector2d> map(map_data,Vector2d(0.0,0.0),map_x,map_y,resolution);

    int robot_num = 5;

    // ROBOT robot(map,task_points,init_points.at(0),robot_num);

    return 0;
}