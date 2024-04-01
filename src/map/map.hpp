#pragma once

#include <iostream>
#include <eigen3/Eigen/Eigen>
using namespace Eigen;

//默认为2d
template <typename T>
class GRID_MAP
{
public:
    uint8_t* mapData = nullptr;
    T origin;
    int mapX,mapY;
    double resolution;
};

template<>
class GRID_MAP<Vector3d>
{
public:
//TODO:换成智能指针
    uint8_t* mapData = nullptr;
    Vector3d origin;
    int mapX,mapY,mapZ;
    double resolution;
};