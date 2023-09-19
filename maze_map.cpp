//
// Created by 魏小爷 on 2023/9/18.
//
#include "maze_map.h"

Map::Map(int _node_num) {
    node_num = _node_num;
    for(int i = 0; i<node_num; i++)
    {
        node.push_back(Node(i));
    }
}

int Map::get_node_num() {
    return node.size();
}

void Map::add_edge(const int &id1, const int &id2, const double &_distance)
{
    // 找到对应节点的迭代器
    auto it_1 = node.begin() + id1;
    auto it_2 = node.begin() + id2;
    bool is_duplicate = false;
    for (Edge &e:it_1->node_edge)
    {
        if(e.dst_id == id2)
        {
            e.distance = _distance;
            cout << "duplicate edge:" << id1 << "," << id2 << "," << _distance << endl;
            is_duplicate = true;
            for (Edge &e_2:it_2->node_edge)
            {
                if(e_2.dst_id == id1)
                {
                    e_2.distance = _distance;
                    break;
                }
            }
            break;
        }
    }
    if(!is_duplicate)
    {
        it_1->node_edge.push_back(Edge(id2,_distance));
        it_2->node_edge.push_back(Edge(id1,_distance));
    }
    return;
}

void Map::del_edge(const int &id1, const int &id2) {
    auto it_1 = node.begin() + id1;
    auto it_2 = node.begin() + id2;
    //
    for (auto it_e = it_1->node_edge.begin(); it_e != it_1->node_edge.end();)
    {
        if(it_e->dst_id == id2)
        {
            it_1->node_edge.erase(it_e);
            continue;
        }
        it_e++;
    }
    for (auto it_e = it_2->node_edge.begin(); it_e != it_2->node_edge.end();)
    {
        if(it_e->dst_id == id1)
        {
            it_2->node_edge.erase(it_e);
            continue;
        }
        it_e++;
    }
}

void Map::dijkstra(const int &src_id, const vector<int> &dst_id) {
    // 未确定的最短路径长度
    vector<double> distance2node(node_num,INF_D);
    // 确定的最短路径长度
    vector<double> min_distance2node(node_num,INF_D);
    // 确定的最短路径长度
    vector<bool> path_find(node_num,false);
    // 上一节点
    vector<int> path2node(node_num,-1);

    // 确定距离当前顶点最近的最短路径
    distance2node.at(src_id) = 0;
    min_distance2node.at(src_id) = 0;
    double min_dis = *min_element(distance2node.begin(), distance2node.end());
    while(min_dis > INF_D+0.1 || min_dis < INF_D - 0.1)
    {
        min_dis = *min_element(distance2node.begin(), distance2node.end());
        // 确定最短路径及长度
        int min_id = min_element(distance2node.begin(), distance2node.end()) - distance2node.begin();
        path_find.at(min_id) = true;
        min_distance2node.at(min_id) = distance2node.at(min_id);

        auto it = node.begin()+min_id;
        // 拓展min_id节点的edge
        for(auto& edge:it->node_edge)
        {
            // 跳过确定的最短路径点
            if(path_find.at(edge.dst_id))
            {
                continue;
            }
            // 如果经过min_id的距离更小，更新该节点的上一节点
            if(edge.distance + min_distance2node.at(min_id) < distance2node.at(edge.dst_id))
            {
                path2node.at(edge.dst_id) = min_id;
                distance2node.at(edge.dst_id) = edge.distance;
            }
        }
        // 不再考虑该节点
        distance2node.at(min_id) = INF_D;
    }

    for(auto &dst:dst_id) // 路径表
    {
        int count = 0;
        cout << src_id << " to " << dst << ":";
        int id = dst;
        // 这里可以补充判断id == src_id
        while(count < node_num && id!= src_id)
        {
            id = path2node.at(id);
            if(id == -1)
            {
                cout << "no path";
                break;
            }
            else
            {
                cout << "<-" << id;
                count++;
            }
        }
        cout << endl;
    }
}
