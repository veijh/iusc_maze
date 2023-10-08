#include <iostream>
// #include "maze_map.h"
#include "gps_coord_tf.h"

using namespace std;
int main() {
    std::cout << "Hello, World!" << std::endl;
    // Map maze(7);
    // maze.add_edge(1,2,12);
    // maze.add_edge(1,3,3);
    // maze.add_edge(1,5,9);
    // maze.add_edge(1,6,10);
    // maze.add_edge(2,4,2);
    // maze.add_edge(2,5,6);
    // maze.add_edge(3,4,2);
    // maze.add_edge(3,6,6);
    // maze.add_edge(4,5,4);
    // maze.add_edge(4,6,7);
    // maze.add_edge(5,6,4);
    // for(auto n = maze.node.begin(); n!=maze.node.end(); n++)
    // {
    //     cout << "node" << n->id << ":";
    //     for(auto e = n->node_edge.begin(); e!=n->node_edge.end(); e++)
    //     {
    //         cout << e->dst_id << "," << e->distance << ";";
    //     }
    //     cout << endl;
    // }
    // vector<int> dst = {0,2,3,4,5,6};
    // maze.dijkstra(1, dst);
    
    Eigen::Vector2d ENU_LALO(39.955225, 116.263203);
    vector<Eigen::Vector2d> MSN_LALO;
    vector<Eigen::Vector2d> MSN_XY;
    MSN_LALO.push_back(Eigen::Vector2d(39.955336, 116.263293));
    MSN_XY.push_back(Eigen::Vector2d(0.0, 20.0));
    MSN_LALO.push_back(Eigen::Vector2d(39.955315, 116.263518));
    MSN_XY.push_back(Eigen::Vector2d(20.0, 20.0));
    MSN_LALO.push_back(Eigen::Vector2d(39.955166, 116.263271));
    MSN_XY.push_back(Eigen::Vector2d(0.0, 0.0));
    MSN_LALO.push_back(Eigen::Vector2d(39.955142, 116.263495));
    MSN_XY.push_back(Eigen::Vector2d(20.0, 0.0));

    GPS_COORD_TF frame_tfer(5.0, ENU_LALO, MSN_LALO, MSN_XY);
    cout << "test 1: " << endl << frame_tfer.MSN_to_ENU(0.0, 0.0) << endl;
    return 0;
}
