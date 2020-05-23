#ifndef DIRECTED_GRAPH_H
#define DIRECTED_GRAPH_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <queue>
#include <stack>
#include <limits>
#include <utility>
#include <algorithm>
#include <string>
#include <list>

using namespace std;

template <typename T>
class vertex
{
public:
    int id;
    T weight;
    vertex(int v_id, T v_weight) : id(v_id), weight(v_weight) {}
};

template <typename T>
class directed_graph
{

private:
    unordered_map<int, T> vertex_weights;               // each element is a pair(id, weight) for a vertex
    unordered_map<int, unordered_map<int, T>> adj_list; // each element is a pair(vertex, the neighbours of this vertex)
                                                        // each neighbour is also a pair (neighbour_vertex, weight for edge from vertex to neighbour_vertex)

public:
    directed_graph();  //A constructor for directed_graph. The graph should start empty.
    ~directed_graph(); //A destructor. Depending on how you do things, this may not be necessary.

    bool contains(const int &);              //Returns true if the graph contains the given vertex_id, false otherwise.
    bool adjacent(const int &, const int &); //Returns true if the first vertex is adjacent to the second, false otherwise.

    void add_vertex(const vertex<T> &);                 //Adds the passed in vertex to the graph (with no edges).
    void add_edge(const int &, const int &, const T &); //Adds a weighted edge from the first vertex to the second.

    void remove_vertex(const int &);            //Removes the given vertex. Should also clear any incident edges.
    void remove_edge(const int &, const int &); //Removes the edge between the two vertices, if it exists.

    size_t in_degree(const int &);  //Returns number of edges coming in to a vertex.
    size_t out_degree(const int &); //Returns the number of edges leaving a vertex.
    size_t degree(const int &);     //Returns the degree of the vertex (both in edges and out edges).

    size_t num_vertices() const; //Returns the total number of vertices in the graph.
    size_t num_edges() const;    //Returns the total number of edges in the graph.

    vertex<T> get_vertex(const int &);
    vector<vertex<T>> get_vertices();                           //Returns a vector containing all the vertices.
    vector<vertex<T>> get_neighbours(const int &);              //Returns a vector containing all the vertices reachable from the given vertex. The vertex is not considered a neighbour of itself.
    vector<vertex<T>> get_second_order_neighbours(const int &); // Returns a vector containing all the second_order_neighbours (i.e., neighbours of neighbours) of the given vertex.
                                                                // A vector cannot be considered a second_order_neighbour of itself.
    bool reachable(const int &, const int &);                   //Returns true if the second vertex is reachable from the first (can you follow a path of out-edges to get from the first to the second?). Returns false otherwise.
    bool contain_cycles() const;                                // Return true if the graph contains cycles (there is a path from any vertices directly/indirectly to itself), false otherwise.

    vector<vertex<T>> depth_first(const int &);   //Returns the vertices of the graph in the order they are visited in by a depth-first traversal starting at the given vertex.
    vector<vertex<T>> breadth_first(const int &); //Returns the vertices of the graph in the order they are visisted in by a breadth-first traversal starting at the given vertex.

    directed_graph<T> out_tree(const int &); //Returns a spanning tree of the graph starting at the given vertex using the out-edges. This means every vertex in the tree is reachable from the root.

    vector<vertex<T>> pre_order_traversal(const int &, directed_graph<T> &);  // returns the vertices in the visiting order of a pre-order traversal of the minimum spanning tree starting at the given vertex.
    vector<vertex<T>> in_order_traversal(const int &, directed_graph<T> &);   // returns the vertices in the visiting order of an in-order traversal of the minimum spanning tree starting at the given vertex.
    vector<vertex<T>> post_order_traversal(const int &, directed_graph<T> &); // returns the vertices in ther visitig order of a post-order traversal of the minimum spanning tree starting at the given vertex.

    vector<vertex<T>> significance_sorting(); // Return a vector containing a sorted list of the vertices in descending order of their significance.
};

template <typename T>
directed_graph<T>::directed_graph() {}

template <typename T>
directed_graph<T>::~directed_graph() {}

/*
	part 1: add, check, remove, retrieve vertex
*/

template <typename T>
void directed_graph<T>::add_vertex(const vertex<T> &vertex)
{
    if (!contains(vertex.id))
    {
        vertex_weights.insert({vertex.id, vertex.weight}); // step 1: add to all_vertices
        adj_list[vertex.id] = unordered_map<int, T>();     // step 2: add to adj_list
    }
}

template <typename T>
bool directed_graph<T>::contains(const int &u_id)
{
    if (vertex_weights.find(u_id) != vertex_weights.end())
    {
        return true;
    }
    return false;
}

// Returns true if the second vertex is reachable from the first
// (can you follow a path of out-edges to get from the first to the second?).
// Returns false otherwise.
template <typename T>
bool directed_graph<T>::reachable(const int &u_id, const int &v_id)
{

    // Get list of all nodes accessible using depth_first
    vector<vertex<T>> path_of_nodes = depth_first(u_id);
    bool reachable = false;

    // Go through path of nodes and look for v_id
    for (auto x : path_of_nodes)
    {
        if (x.id == v_id)
            reachable = true;
    }
    return reachable;
}

template <typename T>
bool directed_graph<T>::contain_cycles() const
{
    return false;
}

template <typename T>
bool directed_graph<T>::adjacent(const int &u_id, const int &v_id)
{
    if (adj_list[u_id].find(v_id) != adj_list[u_id].end())
    {
        return true;
    }
    return false;
}

template <typename T>
size_t directed_graph<T>::num_edges() const
{
    size_t count = 0;
    for (auto &x : adj_list)
    {                             // x == pair<int, unordered_map<int,T>>
        count += x.second.size(); // x.second == unordered_map<int, T>
    }
    return count;
}

template <typename T>
size_t directed_graph<T>::num_vertices() const
{
    return adj_list.size();
}

template <typename T>
size_t directed_graph<T>::degree(const int &u_id)
{
    //degree is all edges associated with that node, hence both out and in degree combined
    return in_degree(u_id) + out_degree(u_id);
}

template <typename T>
size_t directed_graph<T>::out_degree(const int &u_id)
{
    //Out degree is just that nodes edges going outwards which is kept track through adj_list
    return adj_list[u_id].size();
}

template <typename T>
size_t directed_graph<T>::in_degree(const int &u_id)
{
    size_t count = 0;
    //loop through adj_list and count all instances of u_id in each element.second
    for (auto &x : adj_list)
    {
        if (x.second.find(u_id) != x.second.end())
        {
            count++;
        }
    }

    return count;
}

template <typename T>
vector<vertex<T>> directed_graph<T>::get_vertices()
{
    vector<vertex<T>> v;
    for (auto x : vertex_weights)
    {
        v.push_back(vertex<T>(x.first, x.second));
    }
    return v;
}

template <typename T>
vertex<T> directed_graph<T>::get_vertex(const int &u_id)
{
    vertex<T> v(0, 0);
    vector<int>
        vertexIds;
    if (contains(u_id))
    {
        for (auto x : get_vertices()) //Loop through all vertices
        {
            if (x.id == u_id)
            {
                return x;
            }
        }
    }
    return v;
}

template <typename T>
vector<vertex<T>> directed_graph<T>::breadth_first(const int &u_id)
{

    //Create an bool array called visited
    bool *visited = new bool[num_vertices()];
    for (int i = 0; i < num_vertices(); i++)
    {
        visited[i] = false;
    }

    //create a queue
    queue<int> q;

    //visitedNodes will be our output of the traversal
    vector<vertex<T>> visitedNodes;
    int currentNode = u_id;
    visited[currentNode] = true;

    // add first node to queue and visitedNodes
    q.push(currentNode);

    visitedNodes.push_back(get_vertex(currentNode));

    while (!q.empty())
    {
        //set current node to the front of the queue
        currentNode = q.front();
        //dequeue that node
        q.pop();

        //go through all adjacent nodes
        for (auto x : adj_list[currentNode])
        {
            // if we haven't visted a node
            if (!visited[x.first])
            {
                //Enqueue this node and mark it as visited
                visited[x.first] = true;
                q.push(x.first);
                visitedNodes.push_back(get_vertex(x.first));
            }
        }
    }
    return visitedNodes;
}

template <typename T>
directed_graph<T> directed_graph<T>::out_tree(const int &u_id)
{
    //Returns a spanning tree of the graph starting at the given vertex using the out-edges.
    //This means every vertex in the tree is reachable from the root.
    // ---------------------------------------------

    // Find out how many Vertexs there are (we will be creating vertex_count - 1 edges)
    int vertex_total = num_vertices();

    // Initialise
    //   create an empty directed graph with the starting vertex
    //     create directed graph
    directed_graph<T> new_tree;
    new_tree.add_vertex(get_vertex(u_id));
    //     add_vertex(vertex(iud))
    //   For our own tracking
    //      Visit[ ] ; //initialize the visit array to false
    //      Visit[starting vertex]=true ; //make starting vertex visit true
    //      Create a list of edges available and sort by minimal weight
    //           available_edges[]

    // Loop (vertex_count -1)
    //     Go through available_edges(u,v) in order where u in our tree and v not in our tree
    //         Add vertex to our directed graph
    //         Add that edge to our directed graph
    //         Visit[v]=true
    //         remove that edge from available edges
    return new_tree;
}

template <typename T>
vector<vertex<T>> directed_graph<T>::depth_first(const int &u_id)
{
    vector<vertex<T>> visitedNodes;
    stack<vertex<T>> trail; //breadcrumb trail

    //Add first node to stack and visited
    vertex<T> firstNode = get_vertex(u_id);
    trail.push(firstNode);
    visitedNodes.push_back(firstNode);

    // from this point onwards we can always tell which node we are on by trail.top
    vertex<T> currentNode = trail.top();
    //List of neighbours
    vector<vertex<T>> neighbours;
    bool stack_changed = false;
    bool visited = false;

    // While stack not empty
    while (!trail.empty())
    {
        //var get_adjacent_vertexs(to trail.top)
        stack_changed = false;
        currentNode = trail.top();
        neighbours = get_neighbours(currentNode.id);

        //for every neighbour
        for (auto x : neighbours)
        {
            visited = false;
            // loop through visited nodes
            for (auto y : visitedNodes)
            {
                // If we have visited this neigbour then set visited to true
                if (x.id == y.id)
                {
                    visited = true;
                    break;
                }
            }
            // If we have not visited this neigbour then visit it by adding it to trail and visitedNodes
            if (visited == false)
            {
                trail.push(x);
                visitedNodes.push_back(x);
                stack_changed = true;
                break;
            }
        }
        //if we get here then there where no adjacent unvisted nodes left hence we need to go back
        //pop off last entry in stack
        if (stack_changed == false)
        {
            trail.pop();
        }
    }

    return visitedNodes;
}

template <typename T>
void directed_graph<T>::remove_vertex(const int &u_id)
{                               // remove the vertex, as well as all the incident edges
    vertex_weights.erase(u_id); // step 1: remove from all_vertices
    adj_list.erase(u_id);       // step 2: remove from adj_list
    for (auto &x : adj_list)
    {                         // x == pair<int, unordered_map<int,T>>
        x.second.erase(u_id); // x.second == unordered_map<int, T>
    }
}

/*
	part 2: add, check, remove edge
*/

template <typename T>
void directed_graph<T>::add_edge(const int &u_id, const int &v_id, const T &uv_weight)
{
    if (contains(u_id) && contains(v_id))
    { // add the edge only if both vertices are in the graph and the edge is not in the graph
        if (adj_list[u_id].find(v_id) == adj_list[u_id].end())
        {
            adj_list[u_id].insert({v_id, uv_weight});
            // cout << "added edge: " << to_string(u_id) + " --> " + to_string(v_id) << endl;
        }
    }
}

template <typename T>
void directed_graph<T>::remove_edge(const int &u_id, const int &v_id)
{
    if (contains(u_id) && contains(v_id))
    { // remove the edge only if both vertices are in the graph and the edge exists in the graph
        if (adj_list[u_id].find(v_id) != adj_list[u_id].end())
        {
            //edge weight is removed as it is key value pair
            adj_list[u_id].erase(v_id);
            // cout << "removed edge: " << to_string(u_id) + " --> " + to_string(v_id) << endl;
        }
    }
}

template <typename T>
vector<vertex<T>> directed_graph<T>::get_neighbours(const int &u_id)
{
    vector<vertex<T>> v;
    if (contains(u_id))
    { // first make sure the vertex is in the graph
        for (auto x : adj_list[u_id])
        { // adj_list[u_id] is an unordered_map<int, T>
            v.push_back(vertex<T>(x.first, vertex_weights[x.first]));
        }
    }
    return v;
}

template <typename T>
vector<vertex<T>> directed_graph<T>::get_second_order_neighbours(const int &u_id)
{
    vector<vertex<T>> v;
    vector<int> vertexIds;
    if (contains(u_id)) // first make sure the vertex is in the graph
    {
        for (auto x : get_neighbours(u_id)) //loops through the neighbours
        {
            for (auto y : get_neighbours(x.id))
            {
                if (find(vertexIds.begin(), vertexIds.end(), y.id) == vertexIds.end() && u_id != y.id)
                {
                    v.push_back(y);
                    vertexIds.push_back(y.id);
                }
            }
        }
    }
    return v;
}

#endif