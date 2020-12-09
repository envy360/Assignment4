#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>

class Node;
class Edge;

class Graph {
public:
    constexpr static double INF = std::numeric_limits<double>::infinity();

    explicit Graph(bool directed);
    ~Graph();

    // Homework
    bool hasTripletClique() const;
    bool isConnected() const;
    double getMinDistance(const std::string &nid1, const std::string &nid2) const;
    // Optional, extra credit
    double getLongestSimplePath() const;

    // DO NOT CHANGE THE FOLLOWING METHODS
    bool isDirected() const;
    bool addNode(const std::string &nid);
    bool addEdge(const std::string &nid1, const std::string &nid2, double weight);
    std::string toString(const std::string &delimiter = "\n") const;
    void printGraph() const;

private:
    bool directed_;
    std::unordered_map<std::string, Node *> nodes_;
    std::unordered_set<Edge *> edges_;
};

#endif  // GRAPH_HPP