// g++ -std=c++11 graph.cpp test.cpp -o graph_test
// ./graph_test


#include <math.h>
#include <iostream>

#include "graph.hpp"

#ifndef NDEBUG
#define ASSERT(condition, message)                                       \
  do {                                                                   \
    if (!(condition)) {                                                  \
      std::cerr << "Assertion `" #condition "` failed in " << __FILE__   \
                << " line " << __LINE__ << ": " << message << std::endl; \
      std::terminate();                                                  \
    }                                                                    \
  } while (false)
#else
#define ASSERT(condition, message) \
  do {                             \
  } while (false)
#endif

bool isDoubleEqual(double d1, double d2) { return fabs(d1 - d2) <= 0.00001; }

void test_case_1() {
    Graph g(false);

    g.addNode("A");
    g.addNode("B");
    g.addNode("C");

    g.addEdge("A", "B", 1.0);

    bool isConnected = g.isConnected();
    ASSERT(!isConnected, "isConnected = " << (isConnected ? "true" : "false"));

    bool hasTripletClique = g.hasTripletClique();
    ASSERT(!hasTripletClique,
           "hasTripletClique = " << (hasTripletClique ? "true" : "false"));

    double distAB = g.getMinDistance("A", "B");
    ASSERT(isDoubleEqual(distAB, 1.0), "distAB = " << distAB);

    double distAC = g.getMinDistance("A", "C");
    ASSERT(isinf(distAC), "distAC = " << distAC);

    double longestPath = g.getLongestSimplePath();
    ASSERT(isDoubleEqual(longestPath, 1.0), "longestPath = " << longestPath);

    std::cout << "Test case 1 passed" << std::endl;
}

void test_case_2() {
    Graph g(false);

    g.addNode("A");
    g.addNode("B");
    g.addNode("C");

    g.addEdge("A", "B", 1.0);
    g.addEdge("A", "C", 4.0);
    g.addEdge("B", "C", 2.0);

    bool isConnected = g.isConnected();
    ASSERT(isConnected, "isConnected = " << (isConnected ? "true" : "false"));

    bool hasTripletClique = g.hasTripletClique();
    ASSERT(hasTripletClique,
           "hasTripletClique = " << (hasTripletClique ? "true" : "false"));

    double distAC = g.getMinDistance("A", "C");
    ASSERT(isDoubleEqual(distAC, 3.0), "distAC = " << distAC);

    double longestPath = g.getLongestSimplePath();
    ASSERT(isDoubleEqual(longestPath, 6.0), "longestPath = " << longestPath);

    std::cout << "Test case 2 passed" << std::endl;
}

void test_case_3() {
    Graph g(false);

    g.addNode("A");
    g.addNode("B");
    g.addNode("C");
    g.addNode("D");
    g.addNode("E");
    g.addNode("F");
    g.addNode("G");

    g.addEdge("A", "B", 3.0);
    g.addEdge("A", "C", 1.0);
    g.addEdge("A", "D", 5.0);
    g.addEdge("B", "C", 1.0);
    g.addEdge("B", "D", 1.0);
    g.addEdge("C", "D", 4.0);
    g.addEdge("E", "F", 2.0);
    g.addEdge("E", "G", 1.0);
    g.addEdge("F", "G", 3.0);

    bool isConnected = g.isConnected();
    ASSERT(!isConnected, "isConnected = " << (isConnected ? "true" : "false"));

    bool hasTripletClique = g.hasTripletClique();
    ASSERT(hasTripletClique,
           "hasTripletClique = " << (hasTripletClique ? "true" : "false"));

    double distAC = g.getMinDistance("A", "C");
    ASSERT(isDoubleEqual(distAC, 1.0), "distAC = " << distAC);

    double distAD = g.getMinDistance("A", "D");
    ASSERT(isDoubleEqual(distAD, 3.0), "distAD = " << distAD);

    double distAE = g.getMinDistance("A", "E");
    ASSERT(isinf(distAE), "distAE = " << distAE);

    double longestPath = g.getLongestSimplePath();
    ASSERT(isDoubleEqual(longestPath, 12.0), "longestPath = " << longestPath);

    std::cout << "Test case 3 passed" << std::endl;
}

int main() {
    test_case_1();
    test_case_2();
    test_case_3();
    return 0;
}
