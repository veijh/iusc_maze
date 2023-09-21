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
    int dst_id;
    double distance;
    explicit Edge(int _dst_id=-1, double _distance=-1.0): dst_id(_dst_id), distance(_distance){}
};

class Node
{
public:
    int id;
    double x,y;
    vector<Edge> node_edge;
    Node(int _id, double _x = 0.0, double _y = 0.0): id(_id), x(_x), y(_y){}
};

class Map
{
private:
    int node_num;
public:
    bool is_possible;
    vector<Node> node;
    explicit Map(int _node_num = node_total_num);
    int get_node_num();
    void add_edge(const int &id1, const int &id2, const double &_distance);
    void del_edge(const int &id1, const int &id2);
    void dijkstra(const int &src_id, const vector<int> &dst_id);
    void dijkstra(const int &src_id, const vector<int> &dst_id, vector<int> &path);
};

#endif //IUSC_MAZE_MAZE_MAP_H
