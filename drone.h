//
// Created by wjh on 23-9-19.
//

#ifndef IUSC_MAZE_DRONE_H
#define IUSC_MAZE_DRONE_H

#include <iostream>
#include <vector>
#include <list>
#include <algorithm>
#include "maze_map.h"
using namespace std;

class Drone
{
public:
    double current_x,current_y;
    vector<vector<int>> path_list;
    vector<int> merged_path;
    bool is_next_node_reachable();
    void plan(vector<Map> &map_vector, const int &src_id, const vector<int> &dst_id);
    vector<int> get_merged_path();
};
#endif //IUSC_MAZE_DRONE_H
