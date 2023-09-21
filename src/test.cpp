#include <iostream>
#include "maze_map.h"

using namespace std;
int main() {
    std::cout << "Hello, World!" << std::endl;
    Map maze(7);
    maze.add_edge(1,2,12);
    maze.add_edge(1,3,3);
    maze.add_edge(1,5,9);
    maze.add_edge(1,6,10);
    maze.add_edge(2,4,2);
    maze.add_edge(2,5,6);
    maze.add_edge(3,4,2);
    maze.add_edge(3,6,6);
    maze.add_edge(4,5,4);
    maze.add_edge(4,6,7);
    maze.add_edge(5,6,4);
    for(auto n = maze.node.begin(); n!=maze.node.end(); n++)
    {
        cout << "node" << n->id << ":";
        for(auto e = n->node_edge.begin(); e!=n->node_edge.end(); e++)
        {
            cout << e->dst_id << "," << e->distance << ";";
        }
        cout << endl;
    }
    vector<int> dst = {0,2,3,4,5,6};
    maze.dijkstra(1, dst);
    return 0;
}
