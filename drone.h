//
// Created by wjh on 23-9-19.
//

#ifndef IUSC_MAZE_DRONE_H
#define IUSC_MAZE_DRONE_H

#include <iostream>
#include <vector>
#include <list>
#include <algorithm>
#include <cmath>
#include "maze_map.h"
using namespace std;

const double dis_th = 0.3;

class Drone
{
public:
    double cur_x,cur_y;
    double dsr_x, dsr_y;

    int cur_node_id;

    vector<vector<int>> path_list;
    vector<int> merged_path;
    vector<int>::iterator cur_node_ptr;

    Drone(double x=0.0, double y=0.0);
    bool is_next_node_reachable();
    bool is_next_node_reachable(Map &map);
    int plan(vector<Map> &map_vector, const int &src_id, const vector<int> &dst_id);
    vector<int> get_merged_path();
    void fly_to_node(int id, Map &map);
    int next_node();
    int is_reached();
    int is_reached(double x, double y);
};
#endif //IUSC_MAZE_DRONE_H
