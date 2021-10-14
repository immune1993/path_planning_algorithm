//
// Created by zhangxing on 2021/10/8.
//

#ifndef RRT_NODE_H
#define RRT_NODE_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Eigen>
#define inf 1>>20
struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode{
    Eigen::Vector2d coord;//真实坐标
    double cost;//从起始节点到当前节点的cost
    GridNodePtr father;
    double r;//半径

    GridNode(Eigen::Vector2d _coord){
        coord=_coord;
        cost=0;
        r=1;
        father=NULL;
    }
    void drawNode(cv::Mat pic,Eigen::Vector3i color,int thick=-1){
        cv::circle(pic,cv::Point(coord(0),coord(1)),r,cv::Scalar(color(0),color(1),color(2)),thick);
    }
    GridNode(){};
    ~GridNode(){};
};

#endif //RRT_NODE_H
