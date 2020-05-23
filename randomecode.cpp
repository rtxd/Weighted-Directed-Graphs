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
    //   For our own tracking
    //      Visit[ ] ; //initialize the visit array to false
    //      Visit[starting vertex]=true ; //make starting vertex visit true

    // [{A,B,37}, {A,D,16}]
    //* CREATE A LIST OF ALL EDGES AND THEIR WEIGHTS //
    // list of edges adj_list[id] >> returns edges assciated with that id

    // edge
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