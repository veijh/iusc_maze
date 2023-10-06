//
// Created by 魏小爷 on 2023/9/18.
//

#ifndef IUSC_MAZE_MAZE_MAP_H
#define IUSC_MAZE_MAZE_MAP_H

const int node_total_num = 25;
const double INF_D = 999.0;

#include <iostream>
#include <vector>
#include <list>
#include <algorithm>
using namespace std;
class Edge
{
public:
    // 该边的终点；
    int dst_id;
    // 该边的长度
    double distance;
    explicit Edge(int _dst_id=-1, double _distance=-1.0): dst_id(_dst_id), distance(_distance){}
};

class Node
{
public:
    // 该节点的编号
    int id;
    // 该节点的位置
    double x,y;
    // 容器中存放该点的所有边
    vector<Edge> node_edge;
    Node(int _id, double _x = 0.0, double _y = 0.0): id(_id), x(_x), y(_y){}
};

class Map
{
private:
    // 该拓扑地图的总节点数量
    int node_num;
public:
    // 该地图是否可能
    bool is_possible;
    // 容器中存放所有节点信息
    vector<Node> node;
    // 初始化地图；参数为节点数量
    explicit Map(int _node_num = node_total_num);
    // 获取该地图节点总数
    int get_node_num();
    // 添加边
    void add_edge(const int &id1, const int &id2, const double &_distance);
    // 删去边
    void del_edge(const int &id1, const int &id2);
    // dijkstra寻路，终端输出结果
    void dijkstra(const int &src_id, const vector<int> &dst_id);
    // dijkstra寻路，结果存放到path中
    void dijkstra(const int &src_id, const vector<int> &dst_id, vector<int> &path);
};

#endif //IUSC_MAZE_MAZE_MAP_H
