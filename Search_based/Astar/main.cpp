#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "Astar_searcher.h"


int main(){

    double resolution=10;
    Eigen::Vector2d xy_l,xy_u;
    xy_l<<0,0;
    xy_u<<500,500;


    //设置障碍物
    std::vector<Eigen::Vector2d> Obstacle;
    for(double i=5;i<=245;i+=10){
        Eigen::Vector2d pt;
//        pt<<i+125,405;Obstacle.push_back(pt);
//        pt<<i+125,395;Obstacle.push_back(pt);
//        pt<<i+125,415;Obstacle.push_back(pt);
        pt<<205,-i+250;Obstacle.push_back(pt);
        pt<<i+250,205;Obstacle.push_back(pt);
    }
    for(double i=-75;i<=85;i+=10){
        Eigen::Vector2d pt;
        pt<<250,350+i;Obstacle.push_back(pt);
    }

    AstarPathFinder* path_finder=new AstarPathFinder();
    path_finder->initGridMap(resolution,xy_l,xy_u);

    path_finder->setObstacle(Obstacle);

    Eigen::Vector2d start_pt,end_pt;
    start_pt<<5,355;end_pt<<495,355;
    bool haspath=path_finder->AstarGraphSearch(start_pt,end_pt);

    if(haspath){
        std::vector<GridNodePtr> path=path_finder->getPath();

//        for(auto p:path){
//            std::cout<<"("<<p->coord[0]<<" "<<p->coord[1]<<")"<<std::endl;
//        }
    }

    cv::imshow("Astar_Tiebreak",path_finder->picture);
    cv::waitKey();

    delete path_finder;
    return 0;
}
