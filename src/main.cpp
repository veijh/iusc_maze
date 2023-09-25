#include <iostream>
#include <ros/ros.h>
#include "maze_map.h"
#include "drone.h"
#include <string>
#include <cmath>
#include <deque>
#include <cstring>
#include <cstdio>
#include <nav_msgs/Path.h>
#include <tf/tf.h>

using namespace std;
const int real_node_num = 84;
const double offset_x = 0.0-284.3;
const double offset_y = 242.5-400.34;
const double scale = 0.1;

int main_init(Map &maze_template, vector<Map> &maze_vector)
{
    // 从文件中读取maze拓扑
    FILE *maze_topo = fopen("/home/wjh/Documents/iusc_maze/maze_topo.csv", "r");
    FILE *var = fopen("/home/wjh/Documents/iusc_maze/var.csv", "r");

    if(maze_topo == NULL) {
        cout << "fail to open file maze_topo" << endl;
        return 1;
    }

    if(var == NULL) {
        cout << "fail to open file var" << endl;
        return 1;
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
    return 0;
}

void path_to_ros(vector<int> &path, nav_msgs::Path &ros_path, Map &maze_template)
{
    ros_path.header.frame_id = "world";
    ros_path.header.stamp = ros::Time::now();
    ros_path.poses.clear();
    for(auto &node:path)
    {
        geometry_msgs::PoseStamped pos;
        pos.pose.position.x = maze_template.node.at(node).x;
        pos.pose.position.y = maze_template.node.at(node).y;

        pos.header.frame_id = ros_path.header.frame_id;
        pos.header.stamp = ros::Time::now();
        ros_path.poses.push_back(pos);
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "iusc_maze");
    ros::NodeHandle nh("~");
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("planned_path", 1, true);
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>("cur_pose", 1, true);

    ros::Rate loop_rate(10);

    std::cout << "Maze Solver Launch!!" << std::endl;
    
    vector<Map> maze_vector;
    Map maze_template(real_node_num);
    
    if(main_init(maze_template, maze_vector)) return 0;

    //
    vector<int> src = {0,1,2};
    vector<int> dst = {3,4,5};
    vector<int> path;

    geometry_msgs::PoseStamped drone_pose;
    drone_pose.header.frame_id = "world";
    drone_pose.header.stamp = ros::Time::now();

    nav_msgs::Path planned_path;

    double x0 = 0.0, y0 = 0.0;
    nh.getParam("initial_x", x0);
    nh.getParam("initial_y", y0);

    // drone位置的初始化
    Drone drone(x0, y0);

    // 飞向最近的起点
    double min_dis = 999.0;
    int start_id = -1;
    for(int id:src)
    {
        double dx = drone.cur_x - maze_template.node.at(id).x;
        double dy = drone.cur_y - maze_template.node.at(id).y;
        double dis = sqrt(dx*dx+dy*dy);
        if(dis < min_dis)
        {
            min_dis = dis;
            start_id = id;
        }
    }

    drone.fly_to_node(start_id, maze_template);
    /* 理想状态 */
    while(!drone.is_reached())
    {
        drone.cur_x = drone.cur_x + 0.1*cos(drone.dsr_yaw);
        drone.cur_y = drone.cur_y + 0.1*sin(drone.dsr_yaw);
        drone_pose.header.stamp = ros::Time::now();
        drone_pose.pose.position.x = drone.cur_x;
        drone_pose.pose.position.y = drone.cur_y;

        tf::Quaternion q;
        q.setRPY(0, 0, drone.dsr_yaw);
        drone_pose.pose.orientation.x = q.x();
        drone_pose.pose.orientation.y = q.y();
        drone_pose.pose.orientation.z = q.z();
        drone_pose.pose.orientation.w = q.w();
        pos_pub.publish(drone_pose);
        loop_rate.sleep();
    }
    drone.cur_node_id = start_id;

    drone.plan(maze_vector, start_id, dst);
    path_to_ros(drone.merged_path, planned_path, maze_template);
    path_pub.publish(planned_path);

    // 如果规划的路径存在下一个节点
    while(drone.next_node() != -1)
    {
        // 判断下一节点是否可达
        if(drone.is_next_node_reachable(maze_vector.at(1)))
        {
            drone.fly_to_node(drone.next_node(), maze_template);

            /* 理想状态 */
            while(!drone.is_reached())
            {
                drone.cur_x = drone.cur_x + 0.1*cos(drone.dsr_yaw);
                drone.cur_y = drone.cur_y + 0.1*sin(drone.dsr_yaw);
                drone_pose.header.stamp = ros::Time::now();
                drone_pose.pose.position.x = drone.cur_x;
                drone_pose.pose.position.y = drone.cur_y;
                
                tf::Quaternion q;
                q.setRPY(0, 0, drone.dsr_yaw);
                drone_pose.pose.orientation.x = q.x();
                drone_pose.pose.orientation.y = q.y();
                drone_pose.pose.orientation.z = q.z();
                drone_pose.pose.orientation.w = q.w();
                pos_pub.publish(drone_pose);
                loop_rate.sleep();
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

            path_to_ros(drone.merged_path, planned_path, maze_template);
            path_pub.publish(planned_path);

            // 不存在匹配的地图，直接飞向终点
            if(drone.merged_path.empty())
            {
                drone.fly_to_node(3, maze_template);
                /* 理想状态 */
                while(!drone.is_reached())
                {
                    drone.cur_x = drone.dsr_x;
                    drone.cur_y = drone.dsr_y;
                }
                drone.cur_node_id = 3;
            }


        }
    }
    while(ros::ok());
    return 0;
}
