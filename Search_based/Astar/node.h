//
// Created by zhangxing on 2021/9/25.
//

#ifndef ASTAR_NODE_H
#define ASTAR_NODE_H
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Eigen>
#define inf 1>>20
struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode{
    int id;//1->open set,-1->closed set
    Eigen::Vector2d coord;//真实坐标
    Eigen::Vector2i index;//网格坐标
    double gScore,fScore;
    GridNodePtr father;

    GridNode(Eigen::Vector2d _coord,Eigen::Vector2i _index){
        id=0;
        coord=_coord;
        index=_index;
        gScore=inf;
        fScore=inf;
        father=NULL;
    }
    void drawNode(cv::Mat pic,Eigen::Vector3i color,int thick=-1){
        cv::rectangle(pic,cv::Point(coord(0)-5,coord(1)-5),cv::Point(coord(0)+5,coord(1)+5),cv::Scalar(color(0),color(1),color(2)),thick);
    }
    GridNode(){};
    ~GridNode(){};
};

#endif //ASTAR_NODE_H
