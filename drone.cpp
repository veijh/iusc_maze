//
// Created by wjh on 23-9-19.
//
#include "drone.h"

void Drone::plan(vector<Map> &map_vector, const int &src_id, const vector<int> &dst_id) {
    vector<int> path;
    vector<int> possible_map_id;
    vector<int> candidate_path_id;
    if(!path_list.empty())
    {
        path_list.clear();
    }
    for (auto &map:map_vector)
    {
        map.dijkstra(src_id, dst_id, path);
        path_list.push_back(path);
        if(map.is_possible)
        {
            possible_map_id.push_back(&map - &map_vector.at(0));
            candidate_path_id.push_back(&map - &map_vector.at(0));
        }
    }
    // path上第count个点
    int count = 0;
    while(!candidate_path_id.empty())
    {
        vector<int> vote_next_node(map_vector.begin()->get_node_num(),0);
        for(auto id:candidate_path_id)
        {
            int to_node_id = path_list.at(id).at(count);
            vote_next_node.at(to_node_id)++;
        }
        // 选择概率最大的路径点
        merged_path.push_back(max_element(vote_next_node.begin(), vote_next_node.end())-vote_next_node.begin());
        // 删去其他候选路径
        for(int i = 0; i<candidate_path_id.size(); i++)
        {
            if(path_list.at(i).at(count) != merged_path.back())
            {
                candidate_path_id.erase(i+candidate_path_id.begin());
            }
        }

        // 未到达目标
        if(find(dst_id.begin(), dst_id.end(), merged_path.back()) == dst_id.end())
        {
            count++;
        }
        else
        {
            break;
        }
    }
}

