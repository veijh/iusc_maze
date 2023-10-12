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
#include "iusc_maze/Scheme.h"
#include "iusc_maze/Swarm.h"
#include "iusc_maze/Deny.h"
#include "iusc_maze/Dst.h"
#include <boost/bind.hpp>

using namespace std;
const int real_node_num = 83;
const double offset_x = 0.0-181.76;
const double offset_y = 0.0-44.74;
const double scale = 0.1;
double mtg_dist = 1.6;
double flw_dist = 1.6;
double dsr_vel = 2.0;

int main_init(Map &maze_template, vector<Map> &maze_vector, vector<Node> &end_node_vector)
{
    // 从文件中读取maze拓扑
    FILE *maze_topo = fopen("/home/wjh/catkin_ws/src/iusc_maze/maze_topo.csv", "r");
    FILE *var = fopen("/home/wjh/catkin_ws/src/iusc_maze/var.csv", "r");
    FILE *end = fopen("/home/wjh/catkin_ws/src/iusc_maze/end.csv", "r");

    if(maze_topo == NULL) {
        cout << "fail to open file maze_topo" << endl;
        return 1;
    }

    if(var == NULL) {
        cout << "fail to open file var" << endl;
        return 1;
    }

    if(end == NULL) {
        cout << "fail to open file end" << endl;
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

    double end_x = 0.0, end_y = 0.0;
    int count;
    for(int i = 0; i<6; i++)
    {
        fscanf(end, "%lf,%lf", &end_x, &end_y);
        Node node(real_node_num+i, 30.0+end_x, end_y);
        end_node_vector.push_back(node);
        maze_template.node.push_back(node);
    }
    fclose(end);
    return 0;
}

// 将merge_path转换为ros下的path，用于可视化
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

double norm2d(const double &x1, const double &y1, const double &x2, const double &y2)
{
    double dx = x1-x2;
    double dy = y1-y2;
    return sqrt(dx*dx+dy*dy);
}

// 订阅其他无人机的当前节点和目标节点
void scheme_callback(const iusc_maze::Scheme::ConstPtr &scheme, vector<vector<int>> *swarm_scheme)
{
    //cout << scheme->uav_id << ":" << scheme->src_id << "->" << scheme->dst_id << endl;
    swarm_scheme->at(scheme->uav_id).at(0) = scheme->src_id;
    swarm_scheme->at(scheme->uav_id).at(1) = scheme->dst_id;
}

// 订阅其他无人机的位置消息
void swarm_callback(const iusc_maze::Swarm::ConstPtr &swarm, vector<vector<double>> *swarm_info)
{
    //cout << swarm->uav_id << ":" << swarm->x << "," << swarm->y << endl;
    swarm_info->at(swarm->uav_id).at(0) = swarm->x;
    swarm_info->at(swarm->uav_id).at(1) = swarm->y;
}

// 订阅被占据的终点
void dst_callback(const iusc_maze::Dst::ConstPtr &dst, vector<int> *is_occupied)
{
    if(!is_occupied->at(dst->dst_id-real_node_num))
    {
        is_occupied->at(dst->dst_id-real_node_num) = 1;
    }
}

// 订阅排除的方案，replan代表是否需要重新规划
void deny_callback(const iusc_maze::Deny::ConstPtr &deny, vector<Map> *maze_vector, int *replan)
{
    if(maze_vector->at(deny->map_id).is_possible)
    {
        maze_vector->at(deny->map_id).is_possible = false;
        *replan = 1;
    }
    else
    {
        *replan = 0;
    }
}

// 多机协调
int coordinate(Drone &drone, const vector<vector<int>> &swarm_scheme, const vector<vector<double>> &swarm_info, Map &maze_template)
{
    // 是否需要悬停以消解冲突
    int hold = 0;

    // 下一个目标节点
    Node next = maze_template.node.at(drone.next_node());
    // 本机与目标节点的距离
    double distance = norm2d(drone.cur_x, drone.cur_y, next.x, next.y);
    for(int i = 0; i<6; i++)
    {
        if(i == drone.uav_id) continue;
        // 和其他无人机前往相同节点
        if(swarm_scheme.at(i).at(1) == drone.next_node())
        {
            // 目标点一定范围内无人机中，离目标点距离大的悬停
            double other_dis = norm2d(swarm_info.at(i).at(0), swarm_info.at(i).at(1), next.x, next.y);
            if(other_dis < mtg_dist && distance < mtg_dist && other_dis < distance)
            {
                hold = 1;
                cout << "uav " << i << " first, its distance = " << other_dis << endl;
                break;
            }
        }
        // 前往其他机器人当前节点
        if(swarm_scheme.at(i).at(0) == drone.next_node())
        {
            // 不能靠近机器人至一定距离内
            double to_other_dis = norm2d(swarm_info.at(i).at(0), swarm_info.at(i).at(1), drone.cur_x, drone.cur_y);
            if(to_other_dis < flw_dist)
            {
                hold = 1;
                cout << "near uav " << i << " , distance = " << to_other_dis << endl;
                break;
            }
        }
    }
    return hold;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "iusc_maze");
    ros::NodeHandle nh("~");
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("planned_path", 1, true);
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>("cur_pose", 1, true);

    // 多机通信
    // set_target_pos后发布路径方案
    ros::Publisher scheme_pub = nh.advertise<iusc_maze::Scheme>("/scheme", 10, true);
    // 动力学计算前发布本机位置
    ros::Publisher swarm_pub = nh.advertise<iusc_maze::Swarm>("/swarm", 10, true);
    // 前往终点前发布
    ros::Publisher dst_pub = nh.advertise<iusc_maze::Dst>("/dst", 10, true);
    // 方案排除后发布
    ros::Publisher deny_pub = nh.advertise<iusc_maze::Deny>("/deny", 10, true);

    ros::Rate loop_rate(50);
    std::cout << "Maze Solver Launch!!" << std::endl;
    
    vector<Map> maze_vector;
    Map maze_template(real_node_num);
    vector<Node> end_node_vector;

    if(main_init(maze_template, maze_vector, end_node_vector)) return 0;

    //
    vector<int> src = {0,1,2};
    vector<int> dst = {3,4,5};
    vector<int> path;

    geometry_msgs::PoseStamped drone_pose;
    drone_pose.header.frame_id = "world";
    drone_pose.header.stamp = ros::Time::now();

    nav_msgs::Path planned_path;

    iusc_maze::Scheme scheme;
    iusc_maze::Swarm swarm;
    // 不可能的地图
    iusc_maze::Deny map_denied;
    iusc_maze::Dst end_dst;

    map_denied.map_id = -1;
    double x0 = 0.0, y0 = 0.0;
    int uav_id = -1;
    int sim_map_id = -1;
    nh.getParam("initial_x", x0);
    nh.getParam("initial_y", y0);
    nh.getParam("uav_id", uav_id);
    nh.getParam("/sim_map_id", sim_map_id);
    nh.getParam("/mtg_dist", mtg_dist);
    nh.getParam("/flw_dist", flw_dist);
    nh.getParam("/dsr_vel", dsr_vel);

    // drone位置的初始化
    Drone drone(x0, y0, uav_id);

    scheme.uav_id = uav_id;
    scheme.src_id = -1;
    scheme.dst_id = -1;

    swarm.uav_id = uav_id;
    swarm.x = x0;
    swarm.y = y0;

    // 保存的其他无人机位置信息
    vector<vector<double>> swarm_info(6,vector<double>(2,-1.0));
    // 保存的其他无人机路径信息
    vector<vector<int>> swarm_scheme(6, vector<int>(2,-1));
    // 是否需要重规划
    int replan = 0;
    // 待选的目标位置
    vector<int> is_occupied(6, 0);

    ros::Subscriber scheme_sub = nh.subscribe<iusc_maze::Scheme>("/scheme", 50, boost::bind(&scheme_callback, _1, &swarm_scheme));
    ros::Subscriber swarm_sub = nh.subscribe<iusc_maze::Swarm>("/swarm", 50, boost::bind(&swarm_callback, _1, &swarm_info));
    ros::Subscriber dst_sub = nh.subscribe<iusc_maze::Dst>("/dst", 10, boost::bind(&dst_callback, _1, &is_occupied));
    ros::Subscriber deny_sub = nh.subscribe<iusc_maze::Deny>("/deny", 10, boost::bind(&deny_callback, _1, &maze_vector, &replan));

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

    drone.set_target_pos(start_id, maze_template);
    scheme.dst_id = start_id;
    scheme_pub.publish(scheme);

    /* 理想状态 */
    while(!drone.is_reached())
    {
        // 更新无人机状态信息
        // 需要补充一个回调函数，更新cur_x，cur_y

        // 给其他无人机发布本机位置信息
        swarm.x = drone.cur_x;
        swarm.y = drone.cur_y;
        swarm_pub.publish(swarm);

        // 更新swarm_info和swarm_scheme
        ros::spinOnce();

        // 机间避碰协调
        if(coordinate(drone, swarm_scheme, swarm_info, maze_template))
        {
            // 悬停
            drone.dsr_vel = 0;
        }
        else
        {
            // 运动
            drone.dsr_vel = dsr_vel;
        }
        
        // 仿真中的动力学，实际应当被注释
        drone.cur_x = drone.cur_x + drone.dsr_vel*cos(drone.dsr_yaw)*loop_rate.expectedCycleTime().toSec();
        drone.cur_y = drone.cur_y + drone.dsr_vel*sin(drone.dsr_yaw)*loop_rate.expectedCycleTime().toSec();
        
        // 这里发布速度信息给控制器

        // 发布位置用于可视化
        drone_pose.header.stamp = ros::Time::now();
        drone_pose.pose.position.x = drone.cur_x;
        drone_pose.pose.position.y = drone.cur_y;

        // 发布姿态用于可视化
        tf::Quaternion q;
        q.setRPY(0, 0, drone.dsr_yaw);
        drone_pose.pose.orientation.x = q.x();
        drone_pose.pose.orientation.y = q.y();
        drone_pose.pose.orientation.z = q.z();
        drone_pose.pose.orientation.w = q.w();
        pos_pub.publish(drone_pose);

        // 控制发布信息的频率
        loop_rate.sleep();
    }
    // 到达目标后，更新无人机所处节点
    drone.cur_node_id = start_id;

    // 规划到目标点的最短路径
    drone.plan(maze_vector, start_id, dst);
    path_to_ros(drone.merged_path, planned_path, maze_template);
    path_pub.publish(planned_path);

    // 如果规划的路径存在下一个节点
    while(drone.next_node() != -1)
    {
        // 调整航向角判断下一节点是否可达


        // 判断下一节点是否可达
        if(drone.is_next_node_reachable(maze_vector.at(sim_map_id)))
        {
            // 无人机前往下一节点附近
            drone.set_target_pos(drone.next_node(), maze_template);
            scheme.src_id = drone.cur_node_id;
            scheme.dst_id = drone.next_node();
            scheme_pub.publish(scheme);

            while(!drone.is_reached())
            {
                // 更新无人机状态信息
                // 需要补充一个回调函数，更新cur_x，cur_y

                
                swarm.x = drone.cur_x;
                swarm.y = drone.cur_y;
                swarm_pub.publish(swarm);

                // 更新swarm_info和swarm_scheme，以及目标点不可达的replan信息
                ros::spinOnce();

                // 重规划
                if(replan)
                {
                    int before = drone.next_node();
                    drone.plan(maze_vector, drone.cur_node_id, dst);
                    // 重新规划后的下一节点发生变化，则以下一个节点为起点重新规划，避免旧路径终点和新路径起点不连续
                    if(before != drone.next_node())
                    {
                        drone.cur_node_id = before;
                        drone.plan(maze_vector, drone.cur_node_id, dst);
                    }
                    path_to_ros(drone.merged_path, planned_path, maze_template);
                    path_pub.publish(planned_path);
                    break;
                }

                // 机间避碰协调
                if(coordinate(drone, swarm_scheme, swarm_info, maze_template))
                {
                    // 悬停
                    drone.dsr_vel = 0;
                }
                else
                {
                    // 运动
                    drone.dsr_vel = dsr_vel;
                }
                
                // 仿真中的动力学，实际应当被注释
                drone.cur_x = drone.cur_x + drone.dsr_vel*cos(drone.dsr_yaw)*loop_rate.expectedCycleTime().toSec();
                drone.cur_y = drone.cur_y + drone.dsr_vel*sin(drone.dsr_yaw)*loop_rate.expectedCycleTime().toSec();
                
                // 这里发布速度信息给控制器

                // 发布位置用于可视化
                drone_pose.header.stamp = ros::Time::now();
                drone_pose.pose.position.x = drone.cur_x;
                drone_pose.pose.position.y = drone.cur_y;

                // 发布姿态用于可视化
                tf::Quaternion q;
                q.setRPY(0, 0, drone.dsr_yaw);
                drone_pose.pose.orientation.x = q.x();
                drone_pose.pose.orientation.y = q.y();
                drone_pose.pose.orientation.z = q.z();
                drone_pose.pose.orientation.w = q.w();
                pos_pub.publish(drone_pose);

                // 控制发布信息的频率
                loop_rate.sleep();
            }

            // 跳出循环可能原因：1、规划路径改变；2、到达下一节点
            if(replan)
            {
                replan = 0;
            }
            else
            {
                drone.cur_node_id = drone.next_node();
                drone.cur_node_ptr++;
            }
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
                            map_denied.map_id = &map-&maze_vector.at(0);
                            deny_pub.publish(map_denied);
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
                drone.set_target_pos(3, maze_template);
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

    // 飞向最近的终点
    min_dis = 999.0;
    int end_id = -1;
    for(auto &end_node:end_node_vector)
    {
        if(!is_occupied.at(end_node.id-real_node_num))
        {
            double dis = norm2d(drone.cur_x, drone.cur_y,
                end_node.x, end_node.y);
            if(dis < min_dis)
            {
                min_dis = dis;
                end_id = end_node.id;
            }
        }
    }

    end_dst.uav_id = drone.uav_id;
    end_dst.dst_id = end_id;
    dst_pub.publish(end_dst);
    
    drone.merged_path.clear();
    drone.set_target_pos(end_id, maze_template);
    scheme.src_id = drone.cur_node_id;
    scheme.dst_id = drone.next_node();
    scheme_pub.publish(scheme);

    /* 理想状态 */
    while(!drone.is_reached())
    {
        swarm.x = drone.cur_x;
        swarm.y = drone.cur_y;
        swarm_pub.publish(swarm);

        // 更新swarm_info和swarm_scheme
        ros::spinOnce();

        // 飞向终点没有避碰协调
        drone.dsr_vel = dsr_vel;
        
        // 仿真中的动力学
        drone.cur_x = drone.cur_x + drone.dsr_vel*cos(drone.dsr_yaw)*loop_rate.expectedCycleTime().toSec();
        drone.cur_y = drone.cur_y + drone.dsr_vel*sin(drone.dsr_yaw)*loop_rate.expectedCycleTime().toSec();

        // 发布位置用于可视化
        drone_pose.header.stamp = ros::Time::now();
        drone_pose.pose.position.x = drone.cur_x;
        drone_pose.pose.position.y = drone.cur_y;

        // 发布姿态用于可视化
        tf::Quaternion q;
        q.setRPY(0, 0, drone.dsr_yaw);
        drone_pose.pose.orientation.x = q.x();
        drone_pose.pose.orientation.y = q.y();
        drone_pose.pose.orientation.z = q.z();
        drone_pose.pose.orientation.w = q.w();
        pos_pub.publish(drone_pose);
        loop_rate.sleep();
    }
    drone.cur_node_id = end_id;

    // 发布节点信息

    return 0;
}
