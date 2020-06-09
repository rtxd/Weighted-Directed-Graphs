#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <unordered_set>
#include <unordered_map>
#include <set>
#include <array>
#include <list>
#include <forward_list>
#include <deque>
#include <map>
#include <cstddef>
#include <string>
#include <utility>
#include <algorithm>
#include <limits>
// #include <optional>
#include <exception>
#include <stdexcept>

#include "directed_graph.hpp"

using namespace std;

/*
 * Computes the shortest distance from u to v in graph g.
 * The shortest path corresponds to a sequence of vertices starting from u and ends at v,
 * which has the smallest total weight of edges among all possible paths from u to v.
 */
template <typename T>
vector<vertex<T>> shortest_path(directed_graph<T> &graph, int &u_id, int &v_id)
{

    return vector<vertex<T>>();
}

/*
 * Computes the strongly connected components of the graph.
 * A strongly connected component is a subset of the vertices
 * such that for every pair u, v of vertices in the subset,
 * v is reachable from u and u is reachable from v.
 */

template <typename T>
vector<vector<vertex<T>>> strongly_connected_components(directed_graph<T> &graph)
{
    // Create the main stack for SCC
    stack<int> Stack;
    int V = graph.get_vertices().size();
    vector<vertex<T>> verticesInGraph = graph.get_vertices();

    for (int i = 0; i < V; i++)
    {
        cout << "i: " << i << ":" << verticesInGraph[i].id << endl;
    }
    cout << endl;
    cout << endl;

    // Mark all the vertices as not visited (For first DFS)
    bool *visited = new bool[V];
    for (int i = 0; i < V; i++)
    {
        visited[i] = false;
    }

    // Fill vertices in stack according to their finishing times
    for (int i = 0; i < V; i++)
    {
        if (visited[i] == false)
        {
            cout << "Visiting: " << i << endl;
            graph.fillOrder(i, visited, Stack);
        }
    }

    directed_graph<T> gt = graph.getTransposeGraph();

    for (int i = 0; i < V; i++)
    {
        visited[i] = false;
    }

    while (!Stack.empty())
    {
        int node = Stack.top();
        Stack.pop();

        if (!visited[node])
        {
            gt.DFSUtil(node, visited);
        }
    }

    // cout << "The stack is " << Stack.size() << " elements long" << endl;
    // cout << endl;
    // cout << "=========== PRINT STACK ===========" << endl;
    // int stackSize = Stack.size();

    // for (int i = 0; i < stackSize; i++)
    // {
    //     cout << "The stack is " << Stack.size() << " elements long before pop" << endl;
    //     cout << "i: " << i << endl;
    //     int topOfStack = Stack.top();
    //     cout << "Stack top " << verticesInGraph[topOfStack].id << endl;
    //     cout << "pop that off" << endl;
    //     Stack.pop();
    //     cout << "The stack is " << Stack.size() << " elements long after pop" << endl;
    // }

    // // Create a reversed graph
    // Graph gr = getTranspose();

    // // Mark all the vertices as not visited (For second DFS)
    // for (int i = 0; i < V; i++)
    //     visited[i] = false;

    // // Now process all vertices in order defined by Stack
    // while (Stack.empty() == false)
    // {
    //     // Pop a vertex from stack
    //     int v = Stack.top();
    //     Stack.pop();

    //     // Print Strongly connected component of the popped vertex
    //     if (visited[v] == false)
    //     {
    //         gr.DFSUtil(v, visited);
    //         cout << endl;
    //     }
    // }
    return vector<vector<vertex<T>>>();
}

/*
 * Computes a topological ordering of the vertices.
 * For every vertex u in the order, and any of its
 * neighbours v, v appears later in the order than u.
 * You will be given a DAG as the argument.
 */
template <typename T>
vector<vertex<T>> topological_sort(directed_graph<T> &graph)
{

    return vector<vertex<T>>();
}

/*
 * Computes the lowest cost-per-person for delivery over the graph.
 * u is the source vertex, which send deliveries to all other vertices.
 * vertices denote cities; vertex weights denote cities' population;
 * edge weights denote the fixed delivery cost between cities, which is irrelevant to 
 * the amount of goods being delivered. 
 */
template <typename T>
T low_cost_delivery(directed_graph<T> &graph, int &u_id)
{

    return 0;
}
