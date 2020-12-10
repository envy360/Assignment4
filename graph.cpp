#include "graph.hpp"

#include <algorithm>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <vector>
#include <queue>

#include "edge.hpp"
#include "node.hpp"


bool Graph::hasTripletClique() const {
    // Not possible to have TripletClique if less than 3 nodes
    if (nodes_.size() < 3) return false;
    // For each node in the graph
    for (auto it : nodes_) {
        // Get the Node pointer to that node
        Node *node = it.second;
        // To get the number of neighbouring nodes (sizeNeighbour)
        size_t sizeNeighbour = (it.second)->getNeighbors().size();

        if ( sizeNeighbour >= 2 ) {
            // Only if this node has >= 2 neighbour, it can possibly form a triplet
            // std::cout << "neighbourSize = " << sizeNeighbour << "\n";

            // Now get the IDs (name) of all its neighbours into a vector
            std::vector<std::string> neiIDs;
            neiIDs.reserve(sizeNeighbour);
            for (Node *neighbor : node->getNeighbors()) {
                neiIDs.push_back(neighbor->getID());
            }

            // For each node (neiNode) in the focused node's neightbour
            for (size_t i = 0; i < sizeNeighbour; i++) {
                Node *neiNode = nodes_.find(neiIDs[i])->second;
                for (size_t j = i+1; j < sizeNeighbour; j++){
                    // check all the rest of the neighbour nodes to see if they also in the
                    // neiNode's neighbour (if it does than there's a TripletClique
                    if ( (neiNode->getNeighbors()).count(nodes_.find(neiIDs[j])->second) != 0 )
                        return true;
                }
            }
        }
    }
    return false;
}

// Complete this
bool Graph::isConnected() const {
    /*
    bool connected = true;
    for (auto it : nodes_) {
        if ( (it.second)->getNeighbors().size() == 0 ) connected = false;
        // std::cout << (it.second)->getNeighbors().size() << "\n";
    }
    return connected;
     */
    // First degree check: to see if any node not having neighbour
    for (auto it : nodes_) {
        if ( (it.second)->getNeighbors().size() == 0 ) return false;
        // std::cout << (it.second)->getNeighbors().size() << "\n";

        // Set visited = false
        (it.second)->setVisited(false);
    }

    // Doing BFS to see if every node is visited; starting from 1st node in the Graph
    // Get the Node* to the first node in the Adjacent Link List
    Node* snode = (nodes_.begin())->second;
    // Start an empty queue
    queue<Node*> bfsqueue;
    // Set source node visited and Enqueue source node
    snode->setVisited(true);
    bfsqueue.push(snode);
    // BFS
    while ( !bfsqueue.empty()){

    }


}

// Complete this
double Graph::getMinDistance(const std::string &nid1,
                             const std::string &nid2) const {
    assert(nodes_.size() >= 2);  // Must have at least 2 nodes
    // To do
    return INF;
}

// Optional: complete this
double Graph::getLongestSimplePath() const {
    assert(nodes_.size() >= 1);  // Must have at least 1 node
    // To do
    return 0.0;
}

Graph::Graph(bool directed) : directed_(directed){};

Graph::~Graph() {
    for (Edge *edge : edges_) delete edge;
    edges_.clear();
    for (auto it : nodes_) delete it.second;
    nodes_.clear();
}

bool Graph::isDirected() const {
    return directed_;
}

bool Graph::addNode(const std::string &nid) {
    Node *node = new Node(nid);
    bool inserted = nodes_.emplace(nid, node).second;
    if (!inserted) delete node;
    return inserted;
}

bool Graph::addEdge(const std::string &nid1, const std::string &nid2,
                    double weight) {
    auto it1 = nodes_.find(nid1);
    auto it2 = nodes_.find(nid2);
    if (it1 == nodes_.end() || it2 == nodes_.end()) return false;

    Node *node1 = it1->second;
    Node *node2 = it2->second;

    Edge *edge1 = new Edge(node2, weight);
    node1->addEdge(edge1);
    edges_.insert(edge1);
    if (!directed_) {
        Edge *edge2 = new Edge(node1, weight);
        node2->addEdge(edge2);
        edges_.insert(edge2);
    }
    return true;
}

std::string Graph::toString(const std::string &delimiter) const {
    std::vector<std::string> nids;
    nids.reserve(nodes_.size());
    for (auto it : nodes_) nids.push_back(it.first);
    std::sort(nids.begin(), nids.end());

    std::stringstream ss;
    // Iterate each node
    for (size_t i = 0; i < nids.size(); i++) {
        if (i > 0) ss << delimiter;
        ss << nids[i] << ": [";
        Node *node = nodes_.find(nids[i])->second;
        bool first = true;
        for (Node *neighbor : node->getNeighbors()) {
            if (first) {
                ss << neighbor->getID();
                first = false;
            } else {
                ss << ", " << neighbor->getID();
            }
        }
        ss << "]";
    }
    return ss.str();
}

void Graph::printGraph() const { std::cout << toString() << std::endl; }