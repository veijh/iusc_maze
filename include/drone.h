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

// 距离阈值；当与期望位置距离小于该值时，认为到达目标
const double dis_th = 0.3;

class Drone
{
public:
    // 当前位置
    double cur_x, cur_y;
    // 期望位置
    double dsr_x, dsr_y;
    // 期望偏航角
    double dsr_yaw;
    // 期望速度
    double dsr_vel;

    // 该uav的编号
    int uav_id;
    // 当前所处的拓扑节点编号
    int cur_node_id;

    // 存放各个地图下的备选路径
    vector<vector<int>> path_list;
    // 备选路径融合后的执行路径
    vector<int> merged_path;
    // merge_path的迭代器；指明执行到路径中的哪个节点
    vector<int>::iterator cur_node_ptr;

    Drone(double x=0.0, double y=0.0);
    // 位置和无人机编号
    Drone(double x, double y, int id);
    // 判断下一个节点是否可达
    bool is_next_node_reachable();
    // 通过拓扑地图判断下一个节点是否可达（仿真中使用）
    bool is_next_node_reachable(Map &map);
    // 根据map_vector，规划从src到dst的路径，并将结果存放在merged_path中
    int plan(vector<Map> &map_vector, const int &src_id, const vector<int> &dst_id);
    // 暂时没实现
    vector<int> get_merged_path();
    // 设定期望位置
    void set_target_pos(int id, Map &map);
    // 查询路径中的下一个点
    int next_node();
    // 判断是否到达期望位置
    int is_reached();
    // 判断是否到达x,y
    int is_reached(double x, double y);
};
#endif //IUSC_MAZE_DRONE_H
