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

    void display_tree();
    vertex<T> get_vertex(const int &);
    vector<vertex<T>> get_vertices();                           //Returns a vector containing all the vertices.
    vector<vertex<T>> get_neighbours(const int &);              //Returns a vector containing all the vertices reachable from the given vertex. The vertex is not considered a neighbour of itself.
    vector<vertex<T>> get_second_order_neighbours(const int &); // Returns a vector containing all the second_order_neighbours (i.e., neighbours of neighbours) of the given vertex.
                                                                // A vector cannot be considered a second_order_neighbour of itself.
    bool reachable(const int &, const int &);                   //Returns true if the second vertex is reachable from the first (can you follow a path of out-edges to get from the first to the second?). Returns false otherwise.
    bool contain_cycles();                                      // Return true if the graph contains cycles (there is a path from any vertices directly/indirectly to itself), false otherwise.
    bool does_node_contain_cycle(int &);

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
bool directed_graph<T>::contain_cycles()
{

    //Bool containsCycle (var that we return)
    bool containsCycle = false;

    for (int i = 0; i < num_vertices(); i++)
    {
        containsCycle = does_node_contain_cycle(get_vertices()[i].id);
        if (containsCycle)
            return true;
    }

    return containsCycle;
}

template <typename T>
bool directed_graph<T>::does_node_contain_cycle(int &u_id)
{
    //Basically a DFS traversal BUT if node is in visited list AND stack then it's a cycle
    vector<int> visitedList;
    vector<int> nodeStack;

    //Add first node to stack and visited
    int firstNode = u_id;
    nodeStack.push_back(firstNode);
    visitedList.push_back(firstNode);

    //Bool containsCycle (var that we return)
    bool containsCycle = false;

    // from this point onwards we can always tell which node we are on by nodeStack.back
    int currentNode = nodeStack.back();
    //List of neighbours
    vector<vertex<T>> neighbourList;
    bool stack_changed = false;
    bool visited = false;
    bool inStack = false;

    // While stack not empty
    while (!nodeStack.empty())
    {
        //var get_adjacent_vertexs(to nodeStack.back)
        stack_changed = false;
        currentNode = nodeStack.back();
        neighbourList = get_neighbours(currentNode);

        //for every neighbour
        for (auto neighbour : neighbourList)
        {
            visited = find(visitedList.begin(), visitedList.end(), neighbour.id) != visitedList.end();
            inStack = find(nodeStack.begin(), nodeStack.end(), neighbour.id) != nodeStack.end();
            //Is this neighbour in the visited list?
            if (inStack && visited)
            {
                //If in stack AND it's been visited then it's a cycle
                return true;
            }
            else if (!visited)
            {
                // If we have not visited this neigbour then visit it by adding it to trail and visitedNodes
                //Add to stack

                nodeStack.push_back(neighbour.id);
                //Add to visited
                visitedList.push_back(neighbour.id);
                stack_changed = true;
                break;
            }
        }
        //if we get here then there where no adjacent unvisted nodes left hence we need to go back
        //pop off last entry in stack
        if (stack_changed == false)
        {
            nodeStack.pop_back();
        }
    }
    return containsCycle;
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

//Returns a single vertex given a vertex id
template <typename T>
vertex<T> directed_graph<T>::get_vertex(const int &u_id)
{
    vertex<T> v(0, 0);
    vector<int> vertexIds;
    if (contains(u_id))
    {
        for (auto x : get_vertices()) //Loop through all vertices
        {
            //If vertex ID matches then return that vertex
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
    //This will keep track of all visited nodes
    bool *visited = new bool[num_vertices()];
    for (int i = 0; i < num_vertices(); i++)
    {
        visited[i] = false;
    }

    //create a queue
    queue<int> q;

    //the vector "visitedNodes" will be our output of the traversal and is what we return
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

    directed_graph<T> new_tree;

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
    new_tree.add_vertex(get_vertex(u_id)); //start creating tree

    visitedNodes.push_back(get_vertex(currentNode));

    // Keep track of lowest weight edge found
    bool valid_edge_available = false;
    int valid_edge_lowest_weight_vertex;

    double edge_weight;

    while (!q.empty())
    {
        //set current node to the front of the queue

        currentNode = q.front();
        //dequeue that node
        q.pop();

        //go through all adjacent nodes
        for (auto adjacentNode : adj_list[currentNode])
        {

            // keep track of lowest weight vertex
            valid_edge_available = false;
            valid_edge_lowest_weight_vertex = 0;

            // if we haven't visted a node
            if (!visited[adjacentNode.first])
            {
                edge_weight = adjacentNode.second;
                //Enqueue this node and mark it as visited
                visited[adjacentNode.first] = true;
                q.push(adjacentNode.first);
                visitedNodes.push_back(get_vertex(adjacentNode.first));

                new_tree.add_vertex(get_vertex(adjacentNode.first));
                new_tree.add_edge(currentNode, adjacentNode.first, edge_weight);
            }
        }
    }
    return new_tree;
}

template <typename T>
void directed_graph<T>::display_tree()
{
    cout << "display tree:" << endl;

    for (auto node : adj_list)
    {
        cout << "  Vertex: " << node.first << endl;
        for (auto edge : node.second)
        {
            cout << "    Edge: " << node.first << "----(" << edge.second << ")---->" << edge.first << endl;
            //cout << "    Edge Weight (edge.second): " << edge.second << endl;
        }
    }
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

template <typename T>
vector<vertex<T>> directed_graph<T>::pre_order_traversal(const int &u_id, directed_graph<T> &tree)
{
    //We are given a vertex
    // Root/Left/Right
    vector<vertex<T>> visitedNodes;
    stack<vertex<T>> trail; //breadcrumb trail

    //Add first node to stack and visited
    vertex<T> root = get_vertex(u_id);
    trail.push(root);
    visitedNodes.push_back(root);

    // from this point onwards we can always tell which node we are on by trail.top
    vertex<T> currentNode = trail.top();
    //List of neighbours
    vector<vertex<T>> neighbours;
    bool stack_changed = false;
    bool visited = false;

    // While stack not empty
    while (!trail.empty())
    {
        stack_changed = false;
        currentNode = trail.top();
        neighbours = get_neighbours(currentNode.id);

        //for every neighbour
        for (auto neighbour : neighbours)
        {
            visited = false;
            // loop through visited nodes
            for (auto y : visitedNodes)
            {
                // If we have visited this neigbour then set visited to true
                if (neighbour.id == y.id)
                {
                    visited = true;
                    break;
                }
            }
            // If we have not visited this neigbour then visit it by adding it to trail and visitedNodes
            if (visited == false)
            {
                trail.push(neighbour);
                visitedNodes.push_back(neighbour);
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

    // Check that there are no children and just return root

    // v.push_back(vertex<T>(x.first, x.second));
}

template <typename T>
vector<vertex<T>> directed_graph<T>::post_order_traversal(const int &u_id, directed_graph<T> &tree)
{
    //We are given a vertex
    // Left/Root/Right
    vector<vertex<T>> visitedNodes;
    stack<vertex<T>> trail; //breadcrumb trail

    //Add first node to stack and visited
    vertex<T> root = get_vertex(u_id);
    trail.push(root);
    //visitedNodes.push_back(root);

    // from this point onwards we can always tell which node we are on by trail.top
    vertex<T> currentNode = trail.top();
    //List of neighbours
    vector<vertex<T>> neighbours;
    bool stack_changed = false;
    bool visited = false;
    bool reachedBottom = false;

    // While stack not empty
    while (!trail.empty())
    {
        stack_changed = false;
        currentNode = trail.top();
        neighbours = get_neighbours(currentNode.id);

        if (neighbours.empty())
            reachedBottom = true;
        //for every neighbour
        for (auto neighbour : neighbours)
        {
            visited = false;
            // loop through visited nodes
            for (auto visitedNode : visitedNodes)
            {
                // If we have visited this neigbour then set visited to true
                if (neighbour.id == visitedNode.id)
                {
                    visited = true;
                    break;
                }
            }
            // If we have not visited this neigbour then visit it by adding it to trail and visitedNodes
            if (visited == false)
            {
                trail.push(neighbour);
                //If reached the bottom start marking where we've visited

                stack_changed = true;
                break;
            }
        }
        //if we get here then there where no adjacent unvisted nodes left hence we need to go back
        //pop off last entry in stack
        if (stack_changed == false)
        {
            if (reachedBottom)
                visitedNodes.push_back(trail.top());
            trail.pop();
        }
    }

    return visitedNodes;
}

template <typename T>
vector<vertex<T>> directed_graph<T>::in_order_traversal(const int &u_id, directed_graph<T> &tree)
{
    //We are given a vertex
    // Left/Root/Right
    vector<vertex<T>> visitedNodes;
    stack<vertex<T>> trail; //breadcrumb trail

    //Array Status that keeps track of nodes that have been passed
    // if a node is passed twice it's a parent node and needs to be marked as visited
    // 0 = not passed
    // 1 = passed
    // 2 = visited
    int *status = new int[num_vertices()];
    for (int i = 0; i < num_vertices(); i++)
    {
        status[i] = 0;
    }

    //Add first node to stack and visited
    vertex<T> root = get_vertex(u_id);
    trail.push(root);

    // from this point onwards we can always tell which node we are on by trail.top
    vertex<T> currentNode = trail.top();
    //List of neighbours
    vector<vertex<T>> neighbours;
    bool stack_changed = false;
    bool visited = false;
    bool reachedBottom = false;

    // While stack not empty
    while (!trail.empty())
    {
        reachedBottom = false;
        stack_changed = false;
        currentNode = trail.top();
        neighbours = get_neighbours(currentNode.id);

        if (neighbours.empty())
            reachedBottom = true;
        //for every neighbour
        for (auto neighbour : neighbours)
        {
            //Update parents status
            switch (status[currentNode.id])
            {
            case 0:
                //Node has never been passed, but since we're here looking at it's
                // children then we mark as passed
                status[currentNode.id] = 1;
                break;
            case 1:
                //Node has been passed once, this is second time being passed
                // and before we visit next child since we are inorder, we want to record this node
                // 1) push to visited array
                // 2) mark this status as visited
                visitedNodes.push_back(currentNode);
                status[currentNode.id] = 2;

                break;
            }

            visited = false;
            // loop through visited nodes
            for (auto visitedNode : visitedNodes)
            {
                // If we have visited this neigbour then set visited to true
                if (neighbour.id == visitedNode.id)
                {
                    visited = true;
                    break;
                }
            }
            // If we have not visited this neigbour then visit it by adding it to trail and visitedNodes
            if (visited == false)
            {
                trail.push(neighbour);
                //If reached the bottom start marking where we've visited

                stack_changed = true;
                break;
            }
        }
        //if we get here then there where no adjacent unvisted nodes left hence we need to go back
        //pop off last entry in stack
        if (stack_changed == false)
        {
            if (reachedBottom)
                visitedNodes.push_back(trail.top());
            trail.pop();
        }
    }

    return visitedNodes;
}

template <typename T>
vector<vertex<T>> directed_graph<T>::significance_sorting()
{
    vector<vertex<T>> v;
    int temp_index;
    double temp_weight;

    for (auto x : vertex_weights)
    {
        v.push_back(vertex<T>(x.first, x.second));
    }
    int i, j;
    //Bubble Sort on Vertices
    for (i = 0; i < v.size() - 1; i++)
        for (j = 0; j < v.size() - i - 1; j++)
            if (v[j].weight > v[j + 1].weight)
            {
                temp_index = v[j].id;
                temp_weight = v[j].weight;
                v[j].id = v[j + 1].id;
                v[j].weight = v[j + 1].weight;
                v[j + 1].id = temp_index;
                v[j + 1].weight = temp_weight;
            }
    return v;
}

#endif