# Assignment 1 - Weighted Directed Graphs (35%)

## Overview

Your task is to implement a directed_graph class, where each node/edge has a weight.

For example, the following weighted directed graph has five nodes { A, B, C, D, E } and seven edges { (A, B), (A, C), (B, E), (C, D), (D, A), (D, C), (D, E)}. Node C has a weight of 400, and its incident edges: (A, C) has the weight of 9; (D, C) has the weight of 7; (C, D) has the weight of 4.

![alt text](https://static.edusercontent.com/files/tNgXLDUJfwIeh9aIT0dOT3ky.png)

The class should offer a reasonably effective suite of operations. Some (but not all) of basic operations are:

- Adding and removing nodes and edges (with weights);
- Depth-first and breadth-first traversals;
- Computing the minimum spanning tree (MST);
- Pre-order, in-order, and post-order traversals of the MST;

You need to carefully consider:

- Whether it is necessary to declare other classes apart from the directed_graph class.

## The Code

You are provided with a directed_graph.hpp file, which includes most (if not all) of the basic definitions you will need. You made add extra methods, classes, structs, etc., as long as they don't interfere with the existing definitions.

You have also been provided with a main.cpp for ad-hoc testing purposes. main.cpp file does not form part of the assignment, and will not be marked. You can do anything you like with it. When the "run" button is pressed, it will compile and run main.cpp. When the "mark" button is pressed, your code will be run against the tests. Note that the testing code can only mark your code when your code does not cause a program crash (e.g. a segfault). If you get any errors during compiling, make sure you fix that problem first!

Remember to read over all the code before starting.

You have terminal access if you so desire it.

## Directed Graphs

As the abstract data structure, and the possibilities for implementing it, have been covered in the lectures. Some parts of the task may require you to develop some thinking because the lectures may not readily give you the solution. Don't hesitate to ask questions if you are unclear about the task requirements.
