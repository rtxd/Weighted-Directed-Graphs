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
    int INFINITY = 999999;
    vector<vertex<T>> result;
    vector<int> nodeVertexMap;
    int V = graph.get_vertices().size();
    // Create Distance array and set all distances to positive infinity, and 0 for u_id
    int dist[V];
    int prevNode[V];
    // Create visited array and mark all as unvisited
    bool *visited = new bool[V];
    for (int i = 0; i < V; i++)
    {
        visited[i] = false;
        dist[i] = INFINITY; //Infinity
        prevNode[i] = -1;
        cout << "pushing this into node vertex map: " << graph.get_vertices()[i].id << endl;
        nodeVertexMap.push_back(graph.get_vertices()[i].id);
    }
    vector<int>::iterator it = find(nodeVertexMap.begin(), nodeVertexMap.end(), u_id);
    // Set the current vertext to u
    int currentNode = distance(nodeVertexMap.begin(), it);
    // Set the start node distance to value of 0
    dist[currentNode] = 0;

    priority_queue<int, vector<int>, greater<int>> pqNode;
    priority_queue<int, vector<int>, greater<int>> pqDist;
    pqNode.push(currentNode);
    pqDist.push(0);

    // ----- CHECKING EVERTYHING
    for (int i = 0; i < V; i++)
    {
        cout << "For i = " << i << endl;
        cout << "   Visited: " << visited[i] << endl;
        cout << "   Dist: " << dist[i] << endl;
        cout << "   Node Vertex Map: " << nodeVertexMap[i] << endl;
    }
    cout << "int currentNode = " << currentNode << endl;

    // While there are unvisited vertices
    while (!pqNode.empty())
    {
        cout << "While loop start" << endl;

        int index = pqNode.top();
        pqNode.pop();
        pqDist.pop();

        cout << "Index = " << index << endl;

        visited[index] = true;

        // if (pqDist.top() > dist[index])
        for (auto neighbour : graph.get_neighbours(nodeVertexMap[index]))
        {

            cout << "Neigbour value =  " << neighbour.id << endl;
            int neighbourNode = graph.get_node(neighbour.id, nodeVertexMap);

            // if (!visited[neighbourNode])
            // {
            int newDist = dist[index] + graph.get_edge_weight(nodeVertexMap[index], neighbour.id);
            if (newDist < dist[neighbourNode])
            {
                prevNode[neighbourNode] = index;
                dist[neighbourNode] = newDist;
                pqNode.push(neighbourNode);
                pqDist.push(newDist);

                cout << "Neighbour Node: " << neighbourNode << "   newDist: " << newDist << endl;
            }
            // }
        }
    }
    // b4 going further lets see what we got for prevNode

    // From this point we are going to work out path

    vector<int> path;

    // Now we need to find the shortest path dist to v_id (vertex value) ... look this up in neighbourNode (use getNode)
    // We are doing this so that if it's 9999 we can stop now as there is no path

    int FinalNode = graph.get_node(v_id, nodeVertexMap);
    if (dist[FinalNode] == INFINITY)
        return vector<vertex<T>>();
    for (int at = FinalNode; at != -1; at = prevNode[at])
    {
        cout << "at: " << at << endl;
        result.push_back(graph.get_vertex(nodeVertexMap[at]));
    }

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
vector<vector<vertex<T>>> strongly_connected_components(directed_graph<T> &graph)
{
    // Create the main stack for SCC
    stack<int> Stack;
    stack<int> nodeStack;
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
            // cout << "Visiting: " << i << endl;
            graph.fillOrder(i, visited, Stack);
        }
    }

    directed_graph<T> gt = graph.getTransposeGraph();

    for (int i = 0; i < V; i++)
    {
        cout << "i: " << i << ":" << gt.get_vertices()[i].id << endl;
    }

    for (int i = 0; i < V; i++)
    {
        visited[i] = false;
    }

    gt.display_tree();

    //Create Map
    vector<int> nodeVertexMap;
    for (int i = 0; i < V; i++)
    {
        nodeVertexMap.push_back(gt.get_vertices()[i].id);
        cout << "i: " << i << "    value: " << gt.get_vertices()[i].id << endl;
    }

    //go through valueStack and create a new tempNodeStack
    stack<int> tempNodeStack;
    for (int i = 0; i < V; i++)
    { //array find value StackTop
        int stackTop = Stack.top();
        cout << "stacktop" << stackTop << endl;

        vector<int>::iterator it = find(nodeVertexMap.begin(), nodeVertexMap.end(), stackTop);
        //tempNodeStack.push(iterator);
        //cout << "tempNodeStack(push): " << tempNodeStack.top() << endl;
        cout << "tempNodeStack(push): " << distance(nodeVertexMap.begin(), it) << endl;
        tempNodeStack.push(distance(nodeVertexMap.begin(), it));

        Stack.pop();
        // Basically we need to iterate over all the elements of vector and check if given elements exists or not.
        // This can be done in a single line using std::find i.e.
        // Eg. Check if element 22 exists in vector

        //std::vector<int>::iterator it = std::find(vecOfNums.begin(), vecOfNums.end(), 22);

        //vector<int>::iterator it = find(nodeVertexMap.begin(), nodeVertexMap.end(), stackTop);
    }

    cout << "new" << endl;
    // push tempNodeStack into nodeStack
    for (int j = 0; j < V; j++)
    {
        // cout << "hello world ;)" << endl;
        int somerandomname = tempNodeStack.top();
        // cout << somerandomname << endl;
        nodeStack.push(somerandomname);
        tempNodeStack.pop();
        // cout << "nodeStack" << nodeStack.top() << endl;
    }

    //   -----------------------------
    while (!nodeStack.empty())
    {
        int node = nodeStack.top();
        nodeStack.pop();

        if (!visited[node])
        {
            gt.DFSUtil(node, visited);
            cout << endl;
        }
    }

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
    // Result we will return at the end
    vector<vertex<T>> result;
    // Number of vertices in graph
    int N = graph.get_vertices().size();
    // Array of visited
    bool *visited = new bool[N];
    for (int i = 0; i < N; i++)
    {
        visited[i] = false;
    }

    //Create a node vertex map, this map allows us to get the INDEX of a vertex and reference that indexs VALUE and vice versa
    vector<int> nodeVertexMap;
    // Add all vertice IDs to the nodeVertexMap
    for (int i = 0; i < N; i++)
    {
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
            graph.dfs(currentNode, visited, visitedNodes, nodeVertexMap);
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
T low_cost_delivery(directed_graph<T> &graph, int &u_id)
{

    return 0;
}
