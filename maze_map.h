//
// Created by 魏小爷 on 2023/9/18.
//

#ifndef IUSC_MAZE_MAZE_MAP_H
#define IUSC_MAZE_MAZE_MAP_H

const int node_total_num = 25;

#include <iostream>
#include <vector>
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
private:
    int id;
    double position[2];
public:
    vector<edge> node_edge;
    node(int _id, double _x = 0.0, double _y = 0.0);
};

class map
{
private:
    vector<node> n;
    int node_num;
public:
    map();
    int get_node_num();
    void add_edge(int &src_id, int &_dst_id, double &_distance);
};

#endif //IUSC_MAZE_MAZE_MAP_H
