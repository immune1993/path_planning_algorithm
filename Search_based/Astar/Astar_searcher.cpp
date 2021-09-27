//
// Created by zhangxing on 2021/9/25.
//
#include <iostream>
#include "Astar_searcher.h"

bool tie_break = true;
int h_class=3;//1. Euliean,2.Manhadun,3.Diagonal,4.Dijkstra

void AstarPathFinder::initGridMap(double _resolution, Eigen::Vector2d xy_l, Eigen::Vector2d xy_u) {
    picture=cv::Mat::zeros(cv::Size2i(500,500),CV_8SC3);
    picture=cv::Scalar(255,255,255);
    //划分网格
    for(int i=0;i<=500;i+=10){
        cv::line(picture,cv::Point(i,0),cv::Point(i,500),cv::Scalar(0,0,0));
        cv::line(picture,cv::Point(0,i),cv::Point(500,i),cv::Scalar(0,0,0));
    }
    resolution=_resolution;
    xl=xy_l(0),xu=xy_u(0),yl=xy_l(1),yu=xy_u(1);
    GridX_Size=(int)((xu-xl)/resolution);
    GridY_Size=(int)((yu-yl)/resolution);
    GridNodeMap=new GridNodePtr* [GridX_Size];
    for(int i=0;i<GridX_Size;i++){
        GridNodeMap[i]=new GridNodePtr [GridY_Size];
        for(int j=0;j<GridY_Size;j++){
            Eigen::Vector2i tmpIdx;
            tmpIdx<<i,j;
            Eigen::Vector2d pos= gridIndex2coord(tmpIdx);
            GridNodeMap[i][j]=new GridNode(pos,tmpIdx);
        }
    }

}

void AstarPathFinder::setObstacle(std::vector<Eigen::Vector2d> ob) {
    Eigen::Vector3i color(0,0,0);
    for(auto ob_pt:ob){
        Eigen::Vector2i idx= coord2gridIndex(ob_pt);
        GridNodeMap[idx[0]][idx[1]]->id=-1;
        GridNodeMap[idx[0]][idx[1]]->drawNode(picture,color);
    }
}

Eigen::Vector2d AstarPathFinder::gridIndex2coord(const Eigen::Vector2i &index) {
    Eigen::Vector2d pt;
    pt(0)=((double)index(0)+0.5)*resolution+xl;
    pt(1)=((double)index(1)+0.5)*resolution+yl;
    return pt;
}

Eigen::Vector2i AstarPathFinder::coord2gridIndex(const Eigen::Vector2d &coord) {
    Eigen::Vector2i idx;
    idx <<  fmin( fmax( int( (coord(0) - xl) /resolution), 0), GridY_Size - 1),
    fmin( fmax( int( (coord(1) - yl) /resolution), 0), GridY_Size - 1);
    return idx;
}

//计算启发式cost函数
double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2) {
    double h=0;
    //获取坐标差值向量
    auto coord_diff2=node1->coord-node2->coord;
    Eigen::Vector2d coord_diff;
    coord_diff<<abs(coord_diff2[0]),abs(coord_diff2[1]);
    std::vector<double>  diff;
    switch (h_class)
    {
        case 1://Euclidean
        h=sqrt( coord_diff.transpose()*coord_diff);
        break;
        case 2://Manhattan
        h=coord_diff.sum();
        break;
        case 3://Diagonal
        for (int i=0;i<2;i++){diff.push_back(coord_diff[i]);}
        sort(diff.begin(),diff.end());
        h=diff[0]*sqrt(2)+(diff[1]-diff[0]);
        break;
        case 4://Dijkstra
        h=0;
        break;
    }
    if(tie_break){
        double p = 1.0 / 25.0;
        h *= (1.0 + p);
    }
    return h;
}

void AstarPathFinder::getneighbors(GridNodePtr node, std::vector<GridNodePtr> &neighbors,
                                   std::vector<double> &edgecosts) {
    neighbors.clear();
    edgecosts.clear();

    Eigen::Vector2i idx=node->index;
    Eigen::Vector2i incre,tmp;
    for(int i=-1;i<2;i++){
        for(int j=-1;j<2;j++){
            if(i==0&&j==0) continue;
            incre<<i,j;
            tmp=idx+incre;
            if(tmp[0]>=0&&tmp[0]<GridX_Size&&tmp[1]>=0&&tmp[1]<GridY_Size){
                if(GridNodeMap[tmp[0]][tmp[1]]->id!=-1){
                    neighbors.push_back(GridNodeMap[tmp[0]][tmp[1]]);
                    edgecosts.push_back(sqrt(double(incre.transpose()*incre))*resolution);
                }
            }
        }
    }
}

//Astar搜索函数
bool AstarPathFinder::AstarGraphSearch(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt) {
    //坐标转换为网格索引
    Eigen::Vector2i start_idx= coord2gridIndex(start_pt);
    Eigen::Vector2i end_idx= coord2gridIndex(end_pt);
    goalIdx=end_idx;

    //初始化起始和目标节点
    GridNodePtr startNode=GridNodeMap[start_idx(0)][start_idx(1)];
    GridNodePtr endNode=GridNodeMap[end_idx(0)][end_idx(1)];

    //起始和目标节点绘制
    startNode->drawNode(picture,Eigen::Vector3i (0,0,255));
    endNode->drawNode(picture,Eigen::Vector3i (255,0,0));

    openset.clear();

    //将开始节点放入open set
    startNode->gScore=0;
    startNode->fScore= getHeu(startNode,endNode);
    startNode->id=1;//1代表在openset中，-1代表在closedset中
    openset.insert(std::make_pair(startNode->fScore,startNode));

    //存放周边节点和周边路径cost
    std::vector<GridNodePtr> neighbors;
    std::vector<double> edgecosts;
    //当前节点
    GridNodePtr currentNode;

    while(!openset.empty()){
        //取出最小cost节点放入close set
        currentNode=openset.begin()->second;
        openset.erase(openset.begin());
        currentNode->id=-1;

        if(currentNode->index==goalIdx){
            goalNode=currentNode;
            std::cout<<"Path has been found!"<<std::endl;
            return true;
        }

        //获取当前节点周边节点
        getneighbors(currentNode,neighbors,edgecosts);
        //更新周边节点到openset中
        for(int i=0;i<neighbors.size();i++){
            auto tmpNode=neighbors[i];
            //如果节点不在openset中
            if(tmpNode->id==0){
                tmpNode->drawNode(picture,Eigen::Vector3i(0,255,0),1);
                tmpNode->id=1;
                tmpNode->gScore=currentNode->gScore+edgecosts[i];
                tmpNode->fScore=tmpNode->gScore+ getHeu(tmpNode,endNode);
                tmpNode->father=currentNode;
                openset.insert(std::make_pair(tmpNode->fScore,tmpNode));
            }else if(tmpNode->id==1){//如果节点在openset中
                if(tmpNode->gScore>currentNode->gScore+edgecosts[i]){
                    tmpNode->gScore=currentNode->gScore+edgecosts[i];
                    tmpNode->fScore=tmpNode->gScore+ getHeu(tmpNode,endNode);
                    tmpNode->father=currentNode;
                    for(auto p=openset.begin();p!=openset.end();p++){
                        if(p->second==tmpNode){
                            openset.erase(p);
                            break;
                        }
                    }
                    openset.insert(std::make_pair(tmpNode->fScore,tmpNode));
                }
            }
            cv::imshow("Astar",picture);
            cv::waitKey(1);
        }
    }
    std::cout<<"Path has been not found!"<<std::endl;
    return false;
}

std::vector<GridNodePtr> AstarPathFinder::getPath() {
    std::vector<GridNodePtr> path;
    GridNodePtr p=goalNode;
    while(p!=NULL){
        path.push_back(p);
        p=p->father;
    }
    std::reverse(path.begin(),path.end());
    for(int i=1;i<path.size()-1;i++){
        path[i]->drawNode(picture,Eigen::Vector3i(255,0,255));
        cv::imshow("Astar",picture);
        cv::waitKey(1);
    }
    return path;
}

