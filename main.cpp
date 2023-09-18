#include <iostream>
#include "maze_map.h"

using namespace std;
int main() {
    std::cout << "Hello, World!" << std::endl;
    map maze;
    maze.add_edge(1,2,33);
    maze.add_edge(1,2,66);
    maze.add_edge(1,3,4);
    maze.add_edge(2,4,143);
    maze.add_edge(2,1,43);
//    maze.del_edge(1,2);
    for(auto n = maze.n.begin(); n!=maze.n.end(); n++)
    {
        cout << "node" << n->id << ":";
        for(auto e = n->node_edge.begin(); e!=n->node_edge.end(); e++)
        {
            cout << e->dst_id << "," << e->distance << ";";
        }
        cout << endl;
    }
    return 0;
}
