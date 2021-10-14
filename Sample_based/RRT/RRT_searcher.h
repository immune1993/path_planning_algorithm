//
// Created by zhangxing on 2021/10/8.
//

#ifndef RRT_RRT_SEARCHER_H
#define RRT_RRT_SEARCHER_H

#include "node.h"
#include <vector>
#include <cmath>
#include <random>

class RRTPathFinder{
protected:
    std::vector<GridNodePtr> GridNodeSet;//搜索树节点集合
    std::vector<GridNodePtr> ObstacleSet;//障碍物节点集合
    GridNodePtr goalNode;
    double xl,yl,xu,yu;//地图坐标边界

    //std::random_device goal_rd;
    std::mt19937 goal_gen;
    std::uniform_int_distribution<int> goal_dis;

    double getCost(GridNodePtr node1,GridNodePtr node2);//计算两点之间距离
    GridNodePtr getNearestNode(GridNodePtr node);//从搜索树节点中找到最近节点
    GridNodePtr getNextNode(GridNodePtr nearnode,GridNodePtr randnode,double stepsize);//确定next节点
    std::vector<GridNodePtr> getNeighbors(GridNodePtr node,double R);//获取next节点周围半径R内的节点集合
    bool collisioncheck(GridNodePtr node,GridNodePtr nearNode);

public:
    cv::Mat picture;
    cv::VideoWriter write;
    RRTPathFinder(){};
    ~RRTPathFinder(){};
    bool RRTGraphSearch(Eigen::Vector2d start_pt,Eigen::Vector2d end_pt);
    void initGridMap(Eigen::Vector2d xy_l,Eigen::Vector2d xy_u,std::string outVideo);
    void setObstacle(std::vector<Eigen::Vector3d> ob);
    std::vector<GridNodePtr> getPath();

};

#endif //RRT_RRT_SEARCHER_H
