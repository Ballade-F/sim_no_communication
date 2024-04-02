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

    GRID_MAP(uint8_t* map_, T origin_, int map_x, int map_y, double resolution_);
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

    GRID_MAP(uint8_t* map_, Vector3d origin_, int map_x, int map_y, int map_z, double resolution_);
};

template <typename T>
inline GRID_MAP<T>::GRID_MAP(uint8_t *map_, T origin_, int map_x, int map_y, double resolution_)
    :mapData(map_), origin(origin_), mapX(map_x), mapY(map_y), resolution(resolution_)
{
}

inline GRID_MAP<Vector3d>::GRID_MAP(uint8_t *map_, Vector3d origin_, int map_x, int map_y, int map_z, double resolution_)
    :mapData(map_), origin(origin_), mapX(map_x), mapY(map_y), mapZ(map_z), resolution(resolution_)
{
}
