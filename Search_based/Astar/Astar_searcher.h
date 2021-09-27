//
// Created by zhangxing on 2021/9/25.
//

#ifndef ASTAR_ASTAR_SEARCHER_H
#define ASTAR_ASTAR_SEARCHER_H

#include "node.h"
#include <vector>

class AstarPathFinder{
protected:
    GridNodePtr** GridNodeMap;
    Eigen::Vector2i goalIdx;
    GridNodePtr goalNode;
    int GridX_Size,GridY_Size;//网格地图最大索引
    double xl,yl,xu,yu;//地图坐标边界
    double resolution;//网格地图分辨率

    std::multimap<double,GridNodePtr> openset;

    double getHeu(GridNodePtr node1,GridNodePtr node2);
    void getneighbors(GridNodePtr node,std::vector<GridNodePtr> &neighbors,std::vector<double>&edgecosts);
    Eigen::Vector2d gridIndex2coord(const Eigen::Vector2i &index);
    Eigen::Vector2i coord2gridIndex(const Eigen::Vector2d &coord);

public:
    cv::Mat picture;
    AstarPathFinder(){};
    ~AstarPathFinder(){};
    bool AstarGraphSearch(Eigen::Vector2d start_pt,Eigen::Vector2d end_pt);
    void initGridMap(double _resolution,Eigen::Vector2d xy_l,Eigen::Vector2d xy_u);
    void setObstacle(std::vector<Eigen::Vector2d> ob);
    std::vector<GridNodePtr> getPath();

};

#endif //ASTAR_ASTAR_SEARCHER_H
