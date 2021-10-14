#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "RRT_searcher.h"

int main() {

    Eigen::Vector2d xy_l(0,0),xy_u(500,500);

    //设置障碍物
    std::vector<Eigen::Vector3d> Obs;
    for(double i=5;i<=245;i+=10){
        Eigen::Vector3d pt;
        pt<<205,-i+250,10;Obs.push_back(pt);
        pt<<i+250,205,10;Obs.push_back(pt);
    }

    RRTPathFinder* path_finder=new RRTPathFinder();
    path_finder->initGridMap(xy_l,xy_u,"RRT*_hard.avi");

    path_finder->setObstacle(Obs);

    Eigen::Vector2d start_pt,end_pt;
    start_pt<<5,5;end_pt<<480,15;

    bool haspath=path_finder->RRTGraphSearch(start_pt,end_pt);

    if(haspath){
        std::vector<GridNodePtr> path=path_finder->getPath();
    }

    cv::imshow("RRT",path_finder->picture);
    path_finder->write.release();
    cv::waitKey();

    delete path_finder;
    return 0;
}
