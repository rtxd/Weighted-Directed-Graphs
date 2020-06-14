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
vector<vertex<T>> shortest_path(directed_graph<T> graph, int u_id, int v_id)
{
    //Create result vector
    vector<vertex<T>> result;
    // Declare infinity
    const int infinity = 999999;
    // Create node vertex map, this will be used to easily map between the index of
    // a node and it's value
    vector<int> nodeVertexMap;
    int V = graph.get_vertices().size();
    // Create Distance array and set all distances to positive infinity,
    // and 0 for u_id
    int dist[V];
    // Prev node will allow us to trace back our steps at the end so we can get
    // the node path
    int PreviousNode[V];
    // Create visited array
    bool *visited = new bool[V];
    // Loop through all vertex's and set values for arrays
    for (int i = 0; i < V; i++)
    {
        visited[i] = false;
        dist[i] = infinity; //Infinity
        PreviousNode[i] = -1;
        nodeVertexMap.push_back(graph.get_vertices()[i].id);
    }

    // Set the current vertext to u
    int currentNode = graph.get_node(u_id, nodeVertexMap);
    // Set the start node distance to value of 0
    dist[currentNode] = 0;

    // Create two priority queues, one queue to keep track of the node and
    // another queue to keep track of the distance to that node
    priority_queue<int, vector<int>, greater<int>> pqNode;
    priority_queue<int, vector<int>, greater<int>> pqDist;
    // Push currentNode(starting node) and distance (0) into queue
    pqNode.push(currentNode);
    pqDist.push(0);

    // While the queue isn't empty
    while (!pqNode.empty())
    {
        // Set the current index to the minimum value in the queue
        int index = pqNode.top();
        // Pop the min value off the queue
        pqNode.pop();
        pqDist.pop();
        // Mark current node as visited
        visited[index] = true;

        // for each neighbour of this node
        for (auto neighbour : graph.get_neighbours(nodeVertexMap[index]))
        {
            // Get the index of the Neighbour node using our mapping vector
            int neighbourNode = graph.get_node(neighbour.id, nodeVertexMap);

            // Set the new distance = to the old distance plus the
            // edgeweight we just travelled
            int newDist = dist[index] + graph.get_edge_weight(nodeVertexMap[index], neighbour.id);
            if (newDist < dist[neighbourNode])
            {
                PreviousNode[neighbourNode] = index;
                dist[neighbourNode] = newDist;
                // Push node and new distance into queue
                pqNode.push(neighbourNode);
                pqDist.push(newDist);
            }
        }
    }

    // From this point we are going to work out the path we took
    vector<int> path;

    // Now we need to find the shortest path dist to v_id (vertex value) ... look this up in neighbourNode (use getNode)
    int FinalNode = graph.get_node(v_id, nodeVertexMap);
    // We are doing this so that if it's infinity we can stop now as there is no path
    if (dist[FinalNode] == infinity)
        return vector<vertex<T>>();

    // Loop to get the nodes from back to front
    for (int at = FinalNode; at != -1; at = PreviousNode[at])
    {
        // Add each node to the vector
        result.push_back(graph.get_vertex(nodeVertexMap[at]));
    }

    // Reverse it as it'll be in the wrong order
    reverse(result.begin(), result.end());

    return result;
}

/*
 * Computes the strongly connected components of the graph.
 * A strongly connected component is a subset of the vertices
 * such that for every pair u, v of vertices in the subset,
 * v is reachable from u and u is reachable from v.
 */

template <typename T>
vector<vector<vertex<T>>> strongly_connected_components(directed_graph<T> graph)
{
    vector<vector<vertex<T>>> result;
    // Create the main stack for SCC, this stack will be used to create the order
    // in which we will recursively run dfs on each node.
    // Hence this stack is aka the fillOrderStack
    stack<int> fillOrderStack;
    // The stack
    stack<int> nodeStack;
    // Let V = amount of nodes in graph
    int V = graph.get_vertices().size();
    vector<vertex<T>> verticesInGraph = graph.get_vertices();

    // Mark all the vertices as not visited (For first DFS)
    bool *visited = new bool[V];
    for (int i = 0; i < V; i++)
    {
        visited[i] = false;
    }

    for (int i = 0; i < V; i++)
    {
        // Fill vertices in stack according to their finishing times
        if (visited[i] == false)
        {
            graph.fillOrder(i, visited, fillOrderStack);
        }
    }

    // Get the transpose of the graph
    directed_graph<T> gt = graph.get_transpose_graph();

    for (int i = 0; i < V; i++)
    {
        // Reset all nodes to unvisited (so we can reuse this array for our dfs)
        visited[i] = false;
    }

    // Create node vertex map, same functionality as nodeVertexMap
    // in all other functions
    vector<int> nodeVertexMap;
    for (int i = 0; i < V; i++)
    {
        nodeVertexMap.push_back(gt.get_vertices()[i].id);
    }

    // In order to get the values from fillOrderStack into another stack,
    // in the correct order, we need to use a tempNodeStack
    stack<int> tempNodeStack;
    for (int i = 0; i < V; i++)
    {
        int nodeId = graph.get_node(fillOrderStack.top(), nodeVertexMap);
        tempNodeStack.push(nodeId);
        fillOrderStack.pop();
    }

    for (int i = 0; i < V; i++)
    {
        int stackTop = tempNodeStack.top();
        nodeStack.push(stackTop);
        tempNodeStack.pop();
    }

    // Declare an iterator, this will be used to loop through each group of
    // strongly connected components inside the results vector
    int iterator = 0;
    // While node stack is not empty
    while (!nodeStack.empty())
    {
        // set current node equal to the top node, then pop it off stack
        int node = nodeStack.top();
        nodeStack.pop();

        // If we have not visited current node then visit it
        if (!visited[node])
        {
            // Create empty vertex to modify inside dfs
            result.push_back(vector<vertex<T>>());
            gt.dfs_scc(node, visited, result[iterator]);
            // Move on to next group of strongly connected components (add 1 to iterator)
            iterator++;
        }
    }

    return result;
}

/*
 * Computes a topological ordering of the vertices.
 * For every vertex u in the order, and any of its
 * neighbours v, v appears later in the order than u.
 * You will be given a DAG as the argument.
 */
template <typename T>
vector<vertex<T>> topological_sort(directed_graph<T> graph)
{
    // Result we will return at the end
    vector<vertex<T>> result;
    // Number of vertices in graph
    int N = graph.get_vertices().size();
    // Array of visited
    bool *visited = new bool[N];
    //Create a node vertex map, this map allows us to get the INDEX of a vertex and reference that indexs VALUE and vice versa
    vector<int> nodeVertexMap;
    // Add all vertice IDs to the nodeVertexMap and set all nodes to unvisited
    for (int i = 0; i < N; i++)
    {
        visited[i] = false;
        nodeVertexMap.push_back(graph.get_vertices()[i].id);
    }

    // Create an array called ordering, this will store the order of the nodes using their indexs
    int ordering[N];
    // Since topological sort finds the last value first, we will need to start our index from the back of the array
    int i = N - 1;

    // For each node in the graph
    for (int currentNode = 0; currentNode < N; currentNode++)
    {
        // If we have not visited the currentNode then do below
        if (visited[currentNode] == false)
        {
            vector<int> visitedNodes;
            // Run recursive DFS
            // Here we have to pass in the node we are currently visiting, the visited array so we know which nodes have been visited,
            // the visitedNodes vector which will ultimately contain the result this dfs (an output of nodes in topological order),
            // and finally the nodeVertexMap so we can see what node index has which node value
            graph.dfs_topsort(currentNode, visited, visitedNodes, nodeVertexMap);
            // Loop through the visitedNodes and store them in the ordering array in the correct order
            for (auto nodeId : visitedNodes)
            {
                ordering[i] = nodeId;
                i = i - 1;
            }
        }
    }

    // Put the ordered nodes into the returned format (a vector of vertex's aka vector<vertex<T>>)
    for (int i = 0; i < N; i++)
    {
        result.push_back(graph.get_vertex(nodeVertexMap[ordering[i]]));
    }

    return result;
}

/*
 * Computes the lowest cost-per-person for delivery over the graph.
 * u is the source vertex, which send deliveries to all other vertices.
 * vertices denote cities; vertex weights denote cities' population;
 * edge weights denote the fixed delivery cost between cities, which is irrelevant to 
 * the amount of goods being delivered. 
 */
template <typename T>
T low_cost_delivery(directed_graph<T> graph, int u_id)
{
    // Need to keep track of paths (i.e. pairs of vertex) that have been crossed (so we don't visit again)
    // crossed_paths
    // Total_Population
    // Total Cost

    // Cycle through all vertices in any order (may as well be node order)
    // for i = 0 to V-1

    //Only do anything if the vertex is not the starting vertex
    //if i != node(u_id)
    // work our shortest path to this vertex
    // sp  = shortest_path(directed_graph<T> graph, int u_id, int v_id)
    //cycle step

    //

    return 0;
}