#include "directed_graph.hpp"
#include "directed_graph_algorithms.cpp"
#include <stdlib.h>
#include "graph.hpp"

int main()
{

    // Create a graph given in the above diagram
    // Graph g(5);
    // g.addEdge(1, 0);
    // g.addEdge(0, 2);
    // g.addEdge(2, 1);
    // g.addEdge(0, 3);
    // g.addEdge(3, 4);

    // cout << "Following are strongly connected components in "
    //         "given graph \n";
    // g.printSCCs();

    // directed_graph<double> defaultGraph;

    // vertex<double> v0(1, 0.25);
    // vertex<double> v1(2, 1.41);
    // vertex<double> v2(3, 2.32);
    // vertex<double> v3(4, 3.66);
    // vertex<double> v4(5, 4.12);

    // defaultGraph.add_vertex(v0); // add vertex class
    // defaultGraph.add_vertex(v1);
    // defaultGraph.add_vertex(v2);
    // defaultGraph.add_vertex(v3);
    // defaultGraph.add_vertex(v4);

    // defaultGraph.remove_vertex(1); // delete vertex by id

    // cout << "number of vertices: " << defaultGraph.num_vertices() << endl;

    // vector<vertex<double>> vertex_list = defaultGraph.get_vertices();
    // cout << "all vertices: ";
    // for (vertex<double> vt : vertex_list)
    // {
    //     cout << "(" << vt.id << ", " << vt.weight << ") ";
    // }
    // cout << endl;

    // defaultGraph.add_edge(2, 3, 10);
    // defaultGraph.add_edge(2, 4, 20);
    // defaultGraph.add_edge(3, 4, 30);
    // defaultGraph.add_edge(4, 2, 40);
    // defaultGraph.add_edge(4, 3, 50);
    // defaultGraph.add_edge(4, 5, 50);
    // defaultGraph.add_edge(5, 3, 60);

    // defaultGraph.remove_edge(4, 2);

    // defaultGraph.add_edge(4, 2, 40);
    // cout << defaultGraph.adjacent(3, 2) << endl;
    // cout << defaultGraph.degree(3) << endl;

    // // test first order neighbours
    // cout << "all neighbours of 4: ";
    // vector<vertex<double>> neighbour_list = defaultGraph.get_neighbours(4);
    // for (vertex<double> nb : neighbour_list)
    // {
    //     cout << "(" << nb.id << ", " << nb.weight << ") ";
    // }
    // cout << endl;

    // //Test second order neighbours
    // cout << "all second order neighbours of 4: ";
    // vector<vertex<double>> second_order_neighbour_list = defaultGraph.get_second_order_neighbours(4);
    // for (vertex<double> nb : second_order_neighbour_list)
    // {
    //     cout << "(" << nb.id << ", " << nb.weight << ") ";
    // }
    // cout << endl;

    // // test depth first traversal
    // cout << "depth first traversal order: ";
    // vector<vertex<double>> depth_first_traversal = defaultGraph.depth_first(4);
    // for (vertex<double> nb : depth_first_traversal)
    // {
    //     cout << "(" << nb.id << ", " << nb.weight << ") ";
    // }
    // cout << endl;

    // cout << "Is 5 reachable starting at 4? " << defaultGraph.reachable(4, 5) << endl;

    // // test breadth first traversal
    // cout << "breadth first traversal order: ";
    // vector<vertex<double>> breadth_first_traversal = defaultGraph.breadth_first(5);
    // for (vertex<double> nb : breadth_first_traversal)
    // {
    //     cout << "(" << nb.id << ", " << nb.weight << ") ";
    // }
    // cout << endl;

    // directed_graph<double> directedGraph2 = defaultGraph.out_tree(4);

    // cout << "number of vertices: " << directedGraph2.num_vertices() << endl;
    // cout << "number of edges: " << directedGraph2.num_edges() << endl;

    // vector<vertex<double>> sigsort = defaultGraph.significance_sorting();
    // for (vertex<double> nb : sigsort)
    // {
    //     cout << "(" << nb.id << ", " << nb.weight << ") ";
    // }
    // cout << endl;

    // directedGraph2.display_tree();

    // directed_graph<double> simpleTree;

    // vertex<double> vector0(1, 1);
    // vertex<double> vector1(2, 1);
    // vertex<double> vector2(3, 1);
    // vertex<double> vector3(4, 1);
    // vertex<double> vector4(5, 1);
    // vertex<double> vector5(6, 1);
    // vertex<double> vector6(7, 1);

    // simpleTree.add_vertex(vector0); // add vertex class
    // simpleTree.add_vertex(vector1);
    // simpleTree.add_vertex(vector2);
    // simpleTree.add_vertex(vector3);
    // simpleTree.add_vertex(vector4);
    // simpleTree.add_vertex(vector5);
    // simpleTree.add_vertex(vector6);

    // simpleTree.add_edge(1, 2, 1);
    // simpleTree.add_edge(1, 3, 1);
    // simpleTree.add_edge(2, 4, 1);
    // simpleTree.add_edge(2, 5, 1);
    // simpleTree.add_edge(3, 6, 1);
    // simpleTree.add_edge(3, 7, 1);

    // simpleTree.display_tree();

    // vertex_list = simpleTree.in_order_traversal(1, simpleTree);
    // cout << "in order traversal: ";
    // for (vertex<double> vt : vertex_list)
    // {
    //     cout << "(" << vt.id << ", " << vt.weight << ") ";
    // }
    // cout << endl;

    // //Cyclic Graph
    // directed_graph<double> cyclicGraph;

    // cyclicGraph.add_vertex(vertex<double>(1, 1)); // add vertex class
    // cyclicGraph.add_vertex(vertex<double>(2, 1));
    // cyclicGraph.add_vertex(vertex<double>(3, 1));
    // cyclicGraph.add_vertex(vertex<double>(4, 1));
    // cyclicGraph.add_vertex(vertex<double>(5, 1));
    // cyclicGraph.add_vertex(vertex<double>(6, 1));
    // cyclicGraph.add_vertex(vertex<double>(7, 1));

    // cyclicGraph.add_edge(1, 2, 1);
    // cyclicGraph.add_edge(2, 3, 1);
    // cyclicGraph.add_edge(3, 5, 1);
    // cyclicGraph.add_edge(5, 6, 1);
    // cyclicGraph.add_edge(5, 7, 1);
    // cyclicGraph.add_edge(7, 2, 1);
    // cyclicGraph.add_edge(7, 4, 1);

    // cout << simpleTree.contain_cycles() << endl;
    // cout << defaultGraph.contain_cycles() << endl;
    // cout << directedGraph2.contain_cycles() << endl;
    // cout << cyclicGraph.contain_cycles() << endl;

    // directed_graph<double> dg3;

    // dg3.add_vertex(vertex<double>(1, 1));
    // dg3.add_vertex(vertex<double>(2, 1));
    // dg3.add_vertex(vertex<double>(3, 1));
    // dg3.add_vertex(vertex<double>(4, 1));
    // dg3.add_vertex(vertex<double>(5, 1));

    // dg3.add_edge(1, 2, 1);
    // dg3.add_edge(4, 1, 1);
    // dg3.add_edge(3, 5, 1);
    // dg3.add_edge(5, 4, 1);

    // directed_graph<double> dg4;

    // dg4.add_vertex(vertex<double>(1, 1));
    // dg4.add_vertex(vertex<double>(2, 1));
    // dg4.add_vertex(vertex<double>(3, 1));
    // dg4.add_vertex(vertex<double>(4, 1));
    // dg4.add_vertex(vertex<double>(5, 1));

    // dg4.add_edge(1, 4, 1);
    // dg4.add_edge(2, 1, 1);
    // dg4.add_edge(4, 5, 1);
    // dg4.add_edge(5, 2, 1);

    directed_graph<double> dg5;

    dg5.add_vertex(vertex<double>(1, 1));
    dg5.add_vertex(vertex<double>(2, 1));
    dg5.add_vertex(vertex<double>(3, 1));
    dg5.add_vertex(vertex<double>(4, 1));
    dg5.add_vertex(vertex<double>(5, 1));

    dg5.add_edge(1, 2, 1);
    dg5.add_edge(3, 1, 1);
    dg5.add_edge(2, 3, 1);
    dg5.add_edge(3, 4, 1);
    dg5.add_edge(3, 5, 1);

    // directed_graph<double> dg6;

    // dg6.add_vertex(vertex<double>(1, 1));
    // dg6.add_vertex(vertex<double>(2, 1));
    // dg6.add_vertex(vertex<double>(3, 1));
    // dg6.add_vertex(vertex<double>(4, 1));
    // dg6.add_vertex(vertex<double>(5, 1));

    // dg6.add_edge(1, 2, 1);
    // dg6.add_edge(3, 4, 1);
    // dg6.add_edge(4, 1, 1);
    // dg6.add_edge(4, 3, 1);
    // dg6.add_edge(4, 5, 1);
    // dg6.add_edge(5, 2, 1);

    // cout << "dg3 contain cycles: " << dg3.contain_cycles() << endl;
    // cout << "dg4 contain cycles: " << dg4.contain_cycles() << endl;
    cout << "dg5 contain cycles: " << dg5.contain_cycles() << endl;
    // cout << "dg6 contain cycles: " << dg6.contain_cycles() << endl;

    // cout << "reachable: " << directedGraph2.reachable(1, 1) << endl;

    strongly_connected_components(dg5);
    system("pause");
    return 0;
}