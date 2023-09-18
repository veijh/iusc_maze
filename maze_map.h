//
// Created by 魏小爷 on 2023/9/18.
//

#ifndef IUSC_MAZE_MAZE_MAP_H
#define IUSC_MAZE_MAZE_MAP_H

const int node_total_num = 25;

#include <iostream>
#include <vector>
#include <algorithm>
using namespace std;
class edge
{
public:
    int dst_id;
    double distance;
    explicit edge(int _dst_id=-1, double _distance=-1.0): dst_id(_dst_id), distance(_distance){}
};

class node
{
public:
    int id;
    double x,y;
    vector<edge> node_edge;
    node(int _id, double _x = 0.0, double _y = 0.0): id(_id), x(_x), y(_y){}
};

class map
{
private:
    int node_num;
public:
    vector<node> n;
    explicit map(int _node_num = node_total_num);
    int get_node_num();
    void add_edge(const int &id1, const int &id2, const double &_distance);
    void del_edge(const int &id1, const int &id2);
    void dijkstra(const int &src_id, const vector<int> &dst_id);
};

#endif //IUSC_MAZE_MAZE_MAP_H
