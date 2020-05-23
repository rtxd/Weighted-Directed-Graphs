#include "directed_graph.hpp"
#include <stdlib.h>

int main()
{

    directed_graph<double> directedGraph1;

    vertex<double> v0(1, 0.25);
    vertex<double> v1(2, 1.41);
    vertex<double> v2(3, 2.32);
    vertex<double> v3(4, 3.66);
    vertex<double> v4(5, 4.12);

    directedGraph1.add_vertex(v0); // add vertex class
    directedGraph1.add_vertex(v1);
    directedGraph1.add_vertex(v2);
    directedGraph1.add_vertex(v3);
    directedGraph1.add_vertex(v4);

    directedGraph1.remove_vertex(1); // delete vertex by id

    cout << "number of vertices: " << directedGraph1.num_vertices() << endl;

    vector<vertex<double>> vertex_list = directedGraph1.get_vertices();
    cout << "all vertices: ";
    for (vertex<double> vt : vertex_list)
    {
        cout << "(" << vt.id << ", " << vt.weight << ") ";
    }
    cout << endl;

    directedGraph1.add_edge(2, 3, 10);
    directedGraph1.add_edge(2, 4, 20);
    directedGraph1.add_edge(3, 4, 30);
    directedGraph1.add_edge(4, 2, 40);
    directedGraph1.add_edge(4, 3, 50);
    directedGraph1.add_edge(4, 5, 50);
    directedGraph1.add_edge(5, 3, 60);

    directedGraph1.remove_edge(4, 2);

    directedGraph1.add_edge(4, 2, 40);
    cout << directedGraph1.adjacent(3, 2) << endl;
    cout << directedGraph1.degree(3) << endl;

    // test first order neighbours
    cout << "all neighbours of 4: ";
    vector<vertex<double>> neighbour_list = directedGraph1.get_neighbours(4);
    for (vertex<double> nb : neighbour_list)
    {
        cout << "(" << nb.id << ", " << nb.weight << ") ";
    }
    cout << endl;

    //Test second order neighbours
    cout << "all second order neighbours of 4: ";
    vector<vertex<double>> second_order_neighbour_list = directedGraph1.get_second_order_neighbours(4);
    for (vertex<double> nb : second_order_neighbour_list)
    {
        cout << "(" << nb.id << ", " << nb.weight << ") ";
    }
    cout << endl;

    // test depth first traversal
    cout << "depth first traversal order: ";
    vector<vertex<double>> depth_first_traversal = directedGraph1.depth_first(4);
    for (vertex<double> nb : depth_first_traversal)
    {
        cout << "(" << nb.id << ", " << nb.weight << ") ";
    }
    cout << endl;

    cout << "Is 5 reachable starting at 4? " << directedGraph1.reachable(4, 5) << endl;

    // test breadth first traversal
    cout << "breadth first traversal order: ";
    vector<vertex<double>> breadth_first_traversal = directedGraph1.breadth_first(5);
    for (vertex<double> nb : breadth_first_traversal)
    {
        cout << "(" << nb.id << ", " << nb.weight << ") ";
    }
    cout << endl;

    directed_graph<double> directedGraph2 = directedGraph1.out_tree(4);
    cout << "number of vertices: " << directedGraph2.num_vertices() << endl;
    cout << "number of edges: " << directedGraph2.num_edges() << endl;

    directedGraph1.display_tree();
    system("pause");
    return 0;
}