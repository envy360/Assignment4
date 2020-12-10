#ifndef NODE_HPP
#define NODE_HPP

#include <cstdio>
#include <set>
#include <string>

class Edge;

class Node {
public:
    explicit Node(const std::string& id) : id_(id), visited_(false) {
        assert(!id_.empty());
    }

    ~Node() {
        adjacents_.clear();
        neighbors_.clear();
    }

    void setVisited(bool visited) { visited_ = visited; }

    bool isVisited() const { return visited_; }

    bool addEdge(Edge* edge) {
        assert(edge != nullptr);
        if (adjacents_.insert(edge).second) {
            neighbors_.insert(edge->getNode());
        }
        return false;
    }

    const std::string& getID() { return id_; }

    size_t degree() const { return adjacents_.size(); }

    std::set<Edge*> getAdjacencyList() const { return adjacents_; }

    std::set<Node*> getNeighbors() const { return neighbors_; }

private:
    std::string id_;
    bool visited_;
    std::set<Node*> neighbors_;
    std::set<Edge*> adjacents_;
}

#endif  // NODE_HPP
