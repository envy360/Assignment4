#ifndef EDGE_HPP
#define EDGE_HPP

#include<assert.h>

class Node;

class Edge {
public:
    explicit Edge(Node* node, double weight) : node_(node), weight_(weight) {
        assert(node != nullptr);
    }
    ~Edge() {}

    inline void setWeight(double weight) { weight_ = weight; }

    inline Node* getNode() const { return node_; }

    inline double getWeight() const { return weight_; }

private:
    Node* node_;
    double weight_;
};

#endif  // EDGE_HPP
