#include <iostream>
#include <ros/ros.h>
#include "maze_map.h"
#include "drone.h"
#include <string>
#include <cmath>
#include <deque>
#include <cstring>
#include <cstdio>

using namespace std;
int main(int argc, char **argv) {
    std::cout << "Maze Solver Launch!!" << std::endl;
    const int real_node_num = 84;
    const double offset_x = 0.0-284.3;
    const double offset_y = 242.5-400.34;
    const double scale = 0.1;
    vector<Map> maze_vector;
    Map maze_template(real_node_num);

    // 从文件中读取maze拓扑
    FILE *maze_topo = fopen("/home/wjh/Documents/iusc_maze/maze_topo.csv", "r");
    FILE *var = fopen("/home/wjh/Documents/iusc_maze/var.csv", "r");

    if(maze_topo == NULL) {
        cout << "fail to open file maze_topo" << endl;
        return 0;
    }

    if(var == NULL) {
        cout << "fail to open file var" << endl;
        return 0;
    }

    for(int i = 0; i<real_node_num; i++)
    {
        double x = 0.0, y = 0.0;
        int id = 0;
        fscanf(maze_topo,"%lf mm,%lf mm,%d,%*d", &x, &y, &id);
        // 0.1为比例尺
        maze_template.node.at(id).x = scale*(x + offset_x);
        maze_template.node.at(id).y = scale*(y + offset_y);
    }
    int id1 = 0, id2 = 0;
    while(~fscanf(maze_topo,"%*lf mm,%*lf mm,\"%d,%d\",%*d", &id1, &id2))
    {
        // 求距离
        double dx = maze_template.node.at(id1).x-maze_template.node.at(id2).x;
        double dy = maze_template.node.at(id1).y-maze_template.node.at(id2).y;
        double dis = sqrt(dx*dx+dy*dy);
        maze_template.add_edge(id1,id2,dis);
    }
    fclose(maze_topo);

    // 读取可变的拓扑结构
    maze_vector.emplace_back(maze_template);
    auto maze = maze_vector.begin();
    char ju = '\0';
    id1 = 0; id2 = 0;
    // utf8文件开头有 \357\273\277
    fscanf(var,"%*c%*c%*c");
    while(~fscanf(var,"\"%d,%d\"%c", &id1, &id2, &ju))
    {
        // 求距离
        double dx = maze->node.at(id1).x-maze->node.at(id2).x;
        double dy = maze->node.at(id1).y-maze->node.at(id2).y;
        double dis = sqrt(dx*dx+dy*dy);
        maze->add_edge(id1,id2,dis);
        if(ju != ',')
        {
            maze_vector.emplace_back(maze_template);
            maze = maze_vector.end() - 1;
        }
    }
    fclose(var);
    maze_vector.pop_back();
    //
    vector<int> dst = {3,4,5};
    vector<int> path;
//    maze_vector.at(0).dijkstra(0, dst, path);
//    maze_vector.at(1).dijkstra(0, dst, path);
//    maze_vector.at(2).dijkstra(0, dst, path);

    Drone drone;
    drone.plan(maze_vector, 0, dst);

    /*
    for(auto &node:drone.merged_path)
    {
        cout << node;
        if(&node != &drone.merged_path.back())
        {
            cout << "->";
        }
        else
        {
            cout << endl;
        }
    }
    */
    

    // 如果规划的路径存在下一个节点
    while(drone.next_node() != -1)
    {
        // 判断下一节点是否可达
        if(drone.is_next_node_reachable())
        {
            drone.fly_to_node(drone.next_node(), maze_template);

            /* 理想状态 */
            while(!drone.is_reached())
            {
                drone.cur_x = drone.dsr_x;
                drone.cur_y = drone.dsr_y;
            }

            // 实际
            // while(!drone.is_reached()) ros::spinOnce();

            drone.cur_node_id = drone.next_node();
            drone.cur_node_ptr++;
        }
        // 节点不可达
        else
        {
            int possible_map_count = 0;
            // 重新筛选可能的地图
            for(auto &map:maze_vector)
            {
                if(map.is_possible)
                {
                    for(auto &edge:map.node.at(drone.cur_node_id).node_edge)
                    {
                        // next_node不可达，说明不是该地图
                        if(edge.dst_id == drone.next_node())
                        {
                            map.is_possible = false;
                            break;
                        }
                    }
                }
            }
            drone.plan(maze_vector, drone.cur_node_id, dst);
            // 不存在匹配的地图，直接飞向终点
            if(drone.merged_path.empty())
            {
                drone.fly_to_node(3, maze_template);
            }

            /* 理想状态 */
            while(!drone.is_reached())
            {
                drone.cur_x = drone.dsr_x;
                drone.cur_y = drone.dsr_y;
            }
            drone.cur_node_id = 3;
        }
    }
    return 0;
}
