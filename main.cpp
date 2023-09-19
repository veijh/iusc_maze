#include <iostream>
#include "maze_map.h"
#include <string>
#include <cmath>

using namespace std;
int main() {
    std::cout << "Hello, World!" << std::endl;
    const int real_node_num = 84;
    const double offset_x = 0.0-284.3;
    const double offset_y = 242.5-400.34;
    Map maze(real_node_num);
    // 从文件中读取maze拓扑
    FILE *maze_topo = fopen("C:\\Users\\WJH\\CLionProjects\\iusc_maze\\maze_topo.csv", "r");
    if(maze_topo == NULL) {
        cout << "fail to open file" << endl;
        return 0;
    }
    for(int i = 0; i<real_node_num; i++)
    {
        double x = 0.0, y = 0.0;
        int id = 0;
        fscanf(maze_topo,"%lf mm,%lf mm,%d,%*d", &x, &y, &id);
        maze.node.at(id).x = x + offset_x;
        maze.node.at(id).y = y + offset_y;
    }
    int id1 = 0, id2 = 0;
    while(~fscanf(maze_topo,"%*lf mm,%*lf mm,\"%d,%d\",%*d", &id1, &id2))
    {
        // 求距离
        double dx = maze.node.at(id1).x-maze.node.at(id2).x;
        double dy = maze.node.at(id1).y-maze.node.at(id2).y;
        double dis = sqrt(dx*dx+dy*dy);
        maze.add_edge(id1,id2,dis);
    }
    fclose(maze_topo);
    //
    vector<int> dst = {3,4,5};
    maze.dijkstra(0, dst);
    maze.dijkstra(1, dst);
    maze.dijkstra(2, dst);
    return 0;
}
