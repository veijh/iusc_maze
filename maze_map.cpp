//
// Created by 魏小爷 on 2023/9/18.
//
#include "maze_map.h"

node::node(int _id, double _x, double _y) {
    id = _id;
    position[0] = _x;
    position[1] = _y;
}

map::map() {
    node_num = node_total_num;
    for(int i = 0; i<node_num; i++)
    {
        n.push_back(node(i));
    }
}

int map::get_node_num() {
    return n.size();
}
