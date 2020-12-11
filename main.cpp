// g++ -std=c++11 graph.cpp main.cpp -o graph

#include <algorithm>
#include <cstdio>
#include <functional>
#include <iostream>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <assert.h>
using namespace std;
#include "graph.hpp"

// A pair of nodes such that node1 < node2
typedef struct NodePair {
    size_t index1;
    size_t index2;

    NodePair(size_t i, size_t j) {
        assert(i >= 0);
        assert(j >= 0);
        assert(i != j);
        if (i < j) {
            index1 = i;
            index2 = j;
        } else {
            index1 = j;
            index2 = i;
        }
    }
} NodePair;

// Make NodePair hashable
namespace std {
    template <>
    struct hash<NodePair> {
    //    struct std::hash<NodePair> {
        inline size_t operator()(const NodePair& p) const {
            auto hash1 = std::hash<size_t>{}(p.index1);
            auto hash2 = std::hash<size_t>{}(p.index2);
            return hash1 ^ hash2;
        }
    };
}  // namespace std

// Make NodePair hashable
template <>
struct std::equal_to<NodePair> {
    inline bool operator()(const NodePair& a, const NodePair& b) const {
        return a.index1 == b.index1 && a.index2 == b.index2;
    }
};

// Unique node names in sorted order
static std::vector<std::string> NODE_NAMES;
// Unique pairs of nodes as edges with the weights, no self-loop allowed
static std::unordered_map<NodePair, double> NODE_PAIRS;
// Random number using uniform distribution
static std::random_device RD;
static std::mt19937 GEN(RD());
static std::uniform_int_distribution<size_t> DIST;

// Generate n unique node names
static void generateNodeNames(size_t n);
static size_t getRandomIndex();
// Generate m unique pairs of nodes as edges
static void generateNodePairs(size_t m, double weight);

int main() {
    generateNodeNames(100);
    generateNodePairs(500, 1.0);
    generateNodePairs(500, 2.0);
    generateNodePairs(500, 3.0);
    /*
    generateNodeNames(10);
    generateNodePairs(10, 1.0);
     */

    Graph graph(false);
    // Add nodes
    for (std::string name : NODE_NAMES) graph.addNode(name);
    // Add edges
    for (auto it : NODE_PAIRS) {
        graph.addEdge(NODE_NAMES[it.first.index1], NODE_NAMES[it.first.index2],
                      it.second);
    }
    graph.printGraph();

    // TripletClique Test
    cout << endl << endl << "This graph ";
    if( graph.hasTripletClique() ) cout << "has";
    else cout << "does not have";
    cout << " TripletClique !!" << endl;

    return 0;
}

string randomCityName(){

    string consonant[25] = { "b", "c", "d", "f", "g",
                          "h", "j", "k", "l", "m", "n",
                          "p", "q", "r", "s", "t",
                          "v", "w", "x", "y", "z",
                          "ch", "th", "gh", "sh"  };
    string vowel[12] = { "a", "e", "i", "o", "u",
                             "ee", "ae", "ea", "ai", "au", "ou",
                             "oi" };

    string cityname = "";
        cityname = cityname + consonant[rand() % 25]
                  + vowel[rand() % 12]
                  + consonant[rand() %25];
                  // + vowel[rand() % 12]
                  // + consonant[rand() %25] ;

    return cityname;
}

static void generateNodeNames(size_t n) {
    assert(n > 0);
    assert(NODE_NAMES.empty());
    std::unordered_set<std::string> names;
    names.reserve(n);
    while (names.size() < n) {
        std::string name = randomCityName();  // To do: Generate a random name here
        names.insert(name);     // Insert will fail for duplicate names
    }
    NODE_NAMES.reserve(n);
    for (std::string name : names) NODE_NAMES.push_back(name);
    std::sort(NODE_NAMES.begin(), NODE_NAMES.end());
}

static size_t getRandomIndex() {
    assert(!NODE_NAMES.empty());
    if (DIST.a() != 0 || DIST.b() != NODE_NAMES.size() - 1) {
        decltype(DIST.param()) range(0, NODE_NAMES.size() - 1);
        DIST.param(range);
    }
    return DIST(GEN);
}

static void generateNodePairs(size_t m, double weight) {
    assert(m > 0);
    assert(weight > 0.0);
    assert(weight < Graph::INF);

    size_t original = NODE_PAIRS.size();
    NODE_PAIRS.reserve(original + m);
    while (NODE_PAIRS.size() - original < m) {
        // Randomly pick 2 distinct names
        size_t i = getRandomIndex();
        size_t j = getRandomIndex();
        while (i == j) j = getRandomIndex();
        // If a pair of nodes of these 2 names already exists, insert will fail
        if (i < j) {
            NODE_PAIRS.insert({NodePair(i, j), weight});
        } else {
            NODE_PAIRS.insert({NodePair(j, i), weight});
        }
    }
}