#include "graph_searcher.hpp"
#include "opencv2/opencv.hpp"

using cv::Mat;

int main()
{
    gridPathFinder solver;
    Vector3d ol(0,0,0);
    Vector3d ou(0,0,1);
    int max_x = 20, max_y = 10;
    solver.initGridMap(1,ol,ou,max_x,max_y,1);
    solver.setObs(6,0,0);
    solver.setObs(6,1,0);
    solver.setObs(6,2,0);
    solver.setObs(6,3,0);
    solver.setObs(6,4,0);
    solver.setObs(6,5,0);

    solver.setObs(7,5,0);
    solver.setObs(8,5,0);
    solver.setObs(9,5,0);
    solver.setObs(10,5,0);
    solver.setObs(11,5,0);
    solver.setObs(12,5,0);
    solver.setObs(13,5,0);

    Vector3d start(0,0,0);
    Vector3d end(11,3,0);

    solver.graphSearch(start,end,false);
    std::vector<Vector3d> path = solver.getPath();

    Mat gridMap = Mat(max_x,max_y,CV_8UC1,solver.data).clone();
    gridMap *= 255;
    gridMap = 255 - gridMap;

    cv::cvtColor(gridMap,gridMap,cv::COLOR_GRAY2BGR);
    for(auto point : path)
    {
        auto indx = solver.coord2gridIndex(point);
        gridMap.at<cv::Vec3b>(indx.x(),indx.y()) = cv::Vec3b(0,255,0);
    }

    Mat show;
    cv::resize(gridMap,show,cv::Size(640,1280),0,0,cv::INTER_NEAREST);

    cv::imshow("res",show);

    cv::waitKey();

    return 0;
}