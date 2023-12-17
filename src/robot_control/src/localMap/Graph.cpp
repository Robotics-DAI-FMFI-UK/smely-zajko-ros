#include "Graph.h"
#include <limits.h>
#include <queue>

const int INF = INT_MAX;

void Graph::addEdge(int src, int dest, int weight) {
        adjacencyMatrix[src][dest] = weight;
        adjacencyMatrix[dest][src] = weight;  // Assuming an undirected graph
}

void Graph::setXY(int v, int newx, int newy)
{
       x[v] = newx;
       y[v] = newy;
}
        
pair<int, vector<int>> Graph::dijkstra(int start, int end) {
		vector<int> distance(vertices, INF);
		vector<bool> visited(vertices, false);
		vector<int> parent(vertices, -1);
		priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

		distance[start] = 0;
		pq.push({0, start});

		while (!pq.empty()) {
			int u = pq.top().second;
			pq.pop();

			if (visited[u]) continue;
			visited[u] = true;

			for (int v = 0; v < vertices; ++v) {
				if (adjacencyMatrix[u][v] != 0 && !visited[v]) {
					int newDist = distance[u] + adjacencyMatrix[u][v];
					if (newDist < distance[v]) {
						distance[v] = newDist;
						parent[v] = u;
						pq.push({distance[v], v});
					}
				}
			}
		}

		// Reconstruct the path
		
		int current = end;
		vector<int> path;
		while (current != -1) {
			path.push_back(current);
			current = parent[current];			
		}
		reverse(path.begin(), path.end());

		return make_pair(distance[end], path);
}

