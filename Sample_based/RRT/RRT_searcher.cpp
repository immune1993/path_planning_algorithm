//
// Created by zhangxing on 2021/10/8.
//

#include "RRT_searcher.h"

void RRTPathFinder::initGridMap(Eigen::Vector2d xy_l, Eigen::Vector2d xy_u,std::string outVideo) {
    xl=xy_l(0);xu=xy_u(0);yl=xy_l(1);yu=xy_u(1);
    goal_dis=std::uniform_int_distribution<int>(1,100);
    picture=cv::Mat::zeros(cv::Size2i(int(xu-xl),int(yu-yl)),CV_8SC3);
    picture=cv::Scalar(255,255,255);
    write.open(outVideo, CV_FOURCC('M','J','P','G'),60,cv::Size(int(xu-xl),int(yu-yl)), true);
}

void RRTPathFinder::setObstacle(std::vector<Eigen::Vector3d> ob) {
    Eigen::Vector3i color(0,0,0);
    for(auto ob_pt:ob){
        Eigen::Vector2d coo(ob_pt[0],ob_pt[1]);
        GridNodePtr ob_node= new GridNode(coo);
        ob_node->r=ob_pt[2];
        ob_node->drawNode(picture,color);
        ObstacleSet.push_back(ob_node);
    }
}

double RRTPathFinder::getCost(GridNodePtr node1, GridNodePtr node2) {
    auto coord_diff=node1->coord-node2->coord;
    double dis= sqrt(coord_diff.transpose()*coord_diff);
    return dis;
}

GridNodePtr RRTPathFinder::getNearestNode(GridNodePtr node) {
    GridNodePtr tmp=GridNodeSet[0];
    for(auto n:GridNodeSet){
        if (getCost(node,n)< getCost(node,tmp)) tmp=n;
    }
    return tmp;
}

GridNodePtr RRTPathFinder::getNextNode(GridNodePtr nearnode, GridNodePtr randnode, double stepsize) {
    double theta=atan2(randnode->coord(1)-nearnode->coord(1),randnode->coord(0)-nearnode->coord(0));
    Eigen::Vector2d coo(nearnode->coord(0)+stepsize*cos(theta),nearnode->coord(1)+stepsize*sin(theta));
    GridNodePtr nextnode= new GridNode(coo);
    return nextnode;
}

bool RRTPathFinder::collisioncheck(GridNodePtr node,GridNodePtr nearNode) {
    double x2=nearNode->coord(0),y2=nearNode->coord(1);
    double x3=node->coord(0),y3=node->coord(1);
    if(x3<xl||x3>xu||y3<yl||y3>yu) return false;
    double A,B,C;
    if(x2==x3) {A=1;B=0;C=-x2;}
    else if(y2==y3){A=0;B=1;C=-y2;}
    else{A=y2-y3;B=x3-x2;C=x2*y3-y2*x3;}
    for(auto n:ObstacleSet){
        if(getCost(node,n)<=n->r+node->r) return false;
        double x1=n->coord(0),y1=n->coord(1);
        double d=abs(A*x1+B*y1+C)/sqrt(A*A+B*B);
        if(d<=n->r+node->r && (x1-x3)*(x2-x3)+(y1-y3)*(y2-y3)>=0) return false;
    }
    return true;
}

std::vector<GridNodePtr> RRTPathFinder::getNeighbors(GridNodePtr node, double R) {
    std::vector<GridNodePtr> Neighbors;
    for(auto n:GridNodeSet){
        if(getCost(node,n)<R) Neighbors.push_back(n);
    }
    return Neighbors;
}
//RRT/RRT*搜索算法
bool RRTPathFinder::RRTGraphSearch(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt) {
    //初始化起始和目标点
    GridNodePtr startNode=new GridNode(start_pt);startNode->r=5;
    GridNodePtr endNode= new GridNode(end_pt);endNode->r=5;

    //起始和目标点绘制
    startNode->drawNode(picture,Eigen::Vector3i(0,0,255));
    endNode->drawNode(picture,Eigen::Vector3i(255,0,0));

    GridNodeSet.clear();

    //将开始点放入搜索树中
    GridNodeSet.push_back(startNode);
    int count=0;
    srand((unsigned)time(NULL));

    //for(int i=0;i<5000;i++){
    while (1){
        //生成随机点
        count++;
        int x=(rand()%100+1)*5;
        int y=(rand()%100+1)*5;
//        int x=goal_dis(goal_gen)*5;
//        int y=goal_dis(goal_gen)*5;
        Eigen::Vector2d randcoo((double)x,(double)y);
        GridNodePtr randNode =new GridNode(randcoo);

        //获取最近节点
        GridNodePtr nearNode= getNearestNode(randNode);
        //获取next节点
        GridNodePtr nextNode= getNextNode(nearNode,randNode,10);
        //碰撞检测
        if(!collisioncheck(nextNode,nearNode)){
            continue;
        }
        nextNode->father=nearNode;

        //RRT*
        nextNode->cost=nearNode->cost+10;
        //获取next节点周围半径R内的节点集合
        std::vector<GridNodePtr> neighborsNode= getNeighbors(nextNode,25);
        //为next确定父节点
        for(auto n:neighborsNode){
            if(n->cost+getCost(n,nextNode)<nextNode->cost&& collisioncheck(nextNode,n)){
                nextNode->father=n;
                nextNode->cost=n->cost+ getCost(n,nextNode);
            }
        }

        //为周围节点更新父节点是否为nextnode
        for(auto n:neighborsNode){
            if(nextNode->cost+ getCost(nextNode,n)<n->cost&& collisioncheck(n,nextNode)){
                //去除原来连线
                cv::line(picture,cv::Point(n->father->coord(0),n->father->coord(1)),cv::Point(n->coord(0),n->coord(1)),cv::Scalar(255,255,255));
                n->father=nextNode;
                n->cost=nextNode->cost+ getCost(nextNode,n);
                //重新连线
                cv::line(picture,cv::Point(n->father->coord(0),n->father->coord(1)),cv::Point(n->coord(0),n->coord(1)),cv::Scalar(0,0,0));
            }
        }
        //

        nextNode->drawNode(picture,Eigen::Vector3i(0,255,0));
        cv::line(picture,cv::Point(nextNode->father->coord(0),nextNode->father->coord(1)),cv::Point(nextNode->coord(0),nextNode->coord(1)),cv::Scalar(0,0,0));
        GridNodeSet.push_back(nextNode);
        cv::imshow("RRT",picture);
        if(count%15==0) {write.write(picture);std::cout<<count<<std::endl;}
        cv::waitKey(1);
        if(getCost(endNode,nextNode)<5){
            goalNode=nextNode;
            std::cout<<"Path has been found after " + std::to_string(count) +" times search"<<std::endl;
            return true;
        }
    }
    std::cout<<"Path has been not found!"<<std::endl;
    return false;
}

std::vector<GridNodePtr> RRTPathFinder::getPath() {
    std::vector<GridNodePtr> path;
    GridNodePtr p=goalNode;
    while(p!=NULL){
        path.push_back(p);
        p=p->father;
    }
    std::reverse(path.begin(),path.end());
    for(int i=1;i<path.size()-1;i++){
        cv::line(picture,cv::Point(path[i]->coord(0),path[i]->coord(1)),cv::Point(path[i+1]->coord(0),path[i+1]->coord(1)),cv::Scalar(255,0,255),2);
        cv::imshow("RRT",picture);
        write.write(picture);
        cv::waitKey(1);
    }
    return path;
}