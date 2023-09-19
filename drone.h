//
// Created by wjh on 23-9-19.
//

#ifndef IUSC_MAZE_DRONE_H
#define IUSC_MAZE_DRONE_H

#include <iostream>
#include <vector>
#include <algorithm>
using namespace std;

class Drone
{
    double current_x,current_y;
    bool is_next_node_reachable();
};
#endif //IUSC_MAZE_DRONE_H
