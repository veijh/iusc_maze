//
// Created by 魏小爷 on 2023/9/18.
//
#include "maze_map.h"

map::map(int _node_num) {
    node_num = _node_num;
    for(int i = 0; i<node_num; i++)
    {
        n.push_back(node(i));
    }
}

int map::get_node_num() {
    return n.size();
}

void map::add_edge(const int &id1, const int &id2, const double &_distance)
{
    // 找到对应节点的迭代器
    auto it_1 = n.begin() + id1;
    auto it_2 = n.begin() + id2;
    bool is_duplicate = false;
    for (edge &e:it_1->node_edge)
    {
        if(e.dst_id == id2)
        {
            e.distance = _distance;
            cout << "duplicate edge:" << id1 << "," << id2 << "," << _distance << endl;
            is_duplicate = true;
            for (edge &e_2:it_2->node_edge)
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
        it_1->node_edge.push_back(edge(id2,_distance));
        it_2->node_edge.push_back(edge(id1,_distance));
    }
    return;
}

void map::del_edge(const int &id1, const int &id2) {
    auto it_1 = n.begin() + id1;
    auto it_2 = n.begin() + id2;
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

void map::dijkstra(const int &src_id, const vector<int> &dst_id) {
    vector<double> distance2node(node_num,0);
    
}
