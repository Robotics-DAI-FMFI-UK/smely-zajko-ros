#ifndef __GRAPH_H__
#define __GRAPH_H__

#include <vector>
#include <algorithm>

using namespace std;


class Graph {
	
public:
    int vertices;
    vector<int> x, y;
    vector<vector<int>> adjacencyMatrix;

    Graph(int v = 0) : vertices(v), adjacencyMatrix(v, vector<int>(v, 0)), x(v, 0), y(v, 0) {}

    void addEdge(int src, int dest, int weight);
    void setXY(int v, int newx, int newy);
    pair<int, vector<int>> dijkstra(int start, int end);
};

#endif
