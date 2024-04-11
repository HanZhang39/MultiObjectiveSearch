#pragma once

#include <cstddef>
#include <memory>
#include "common.h"

// Structs and classes
struct Edge {
    size_t          source;
    size_t          target;
    std::vector<cost_t>    cost;
    std::vector<cost_t>    apex;

    Edge(size_t source, size_t target, std::vector<cost_t> cost) : source(source), target(target), cost(cost), apex(cost) {}
    Edge(size_t source, size_t target, std::vector<cost_t> cost, std::vector<cost_t> apex) : source(source), target(target), cost(cost), apex(apex) {}
    Edge inverse() const {
        return Edge(this->target, this->source, this->cost, this->apex);
    }
};
std::ostream& operator<<(std::ostream &stream, const Edge &edge);


// Graph representation as adjacency matrix
class AdjacencyMatrix {
private:
    size_t num_of_objectives = 0;

public:
    std::vector<std::vector<Edge>> matrix;

    AdjacencyMatrix() = default;
    AdjacencyMatrix(size_t graph_size, std::vector<Edge> &edges, bool inverse=false);
    void add(Edge edge);
    size_t size(void) const;
    size_t get_num_of_objectives() const;
    std::vector<Edge>& operator[](size_t vertex_id);
    const std::vector<Edge> &operator[](size_t vertex_id) const;

    friend std::ostream& operator<<(std::ostream &stream, const AdjacencyMatrix &adj_matrix);

    void update_edges(size_t state, std::vector<Edge>& edges){
        matrix[state] = edges;
    }

    void update_edge(size_t state, int idx, Edge edge){
        assert(idx < matrix[state].size());
        matrix[state][idx] = edge;
    }

    void add_edge(Edge& edge){
        matrix[edge.source].push_back(edge);
    }
};

struct ApexCostPair {
    std::vector<cost_t> apex;
    std::vector<cost_t> cost;

    ApexCostPair(const std::vector<cost_t> cost): apex(cost), cost(cost){}
    ApexCostPair(const std::vector<cost_t> apex, const std::vector<cost_t> cost): apex(apex), cost(cost){}
    ApexCostPair(const ApexCostPair& ap1, const ApexCostPair & ap2): apex(ap1.apex), cost(ap1.cost){
        for (size_t i = 0; i < ap1.apex.size(); i++){
            apex[i] += ap2.apex.at(i);
            cost[i] += ap2.cost.at(i);
        }
    }
};


struct ApexCostPairCmp{
    
    bool operator()(const ApexCostPair& a, const ApexCostPair& b)
    {
        // smallest comes first
        return a.apex < b.apex;
    }
};


class MultiCostEdge {
public:
    size_t          source;
    size_t          target;

    // std::vector<std::vector<cost_t>> costs;
    std::vector<ApexCostPair> costs;
    std::vector<cost_t> min_cost;

    MultiCostEdge(size_t source, size_t target,
                  std::vector<ApexCostPair> costs) :
        source(source), target(target), costs(costs) {
        if (costs.size() > 0){
            min_cost = costs[0].apex;
            for (int i = 1; i < costs.size(); i++){
                for (int j = 0; j < min_cost.size(); j++){
                min_cost[j] = min(costs[i].apex[j], min_cost[j]);
                }
            }

        }
    };


    MultiCostEdge() {};

    ~MultiCostEdge() {};

    MultiCostEdge(size_t source, size_t target, std::vector<cost_t> cost) :
        source(source), target(target), costs({ cost }), min_cost(cost)
    {};

    void insert(std::vector<cost_t> cost);

    void insert(const Edge & edge);

    void sort(){
        std::sort(costs.begin(), costs.end(), ApexCostPairCmp());
    };

    MultiCostEdge inverse() const {
        return MultiCostEdge(this->target, this->source, this->costs);
    }
};

class AdjacencyMultiCostMatrix {
private:
    size_t num_of_objectives = 0;

public:
    std::vector<std::vector<shared_ptr<MultiCostEdge>>> matrix;
    std::vector<std::vector<shared_ptr<MultiCostEdge>>> inv_matrix;


    size_t get_num_incident_edges(size_t state){
        size_t cnt = 0;
        for (auto & ptr: matrix[state]){
            cnt += ptr->costs.size();
        }
        for (auto & ptr: inv_matrix[state]){
            cnt += ptr->costs.size();
        }
        return cnt;
    }

    AdjacencyMultiCostMatrix() = default;

    AdjacencyMultiCostMatrix(size_t graph_size, std::vector<Edge> &edges);

    void add(const Edge & edge);
    void add(const MultiCostEdge & edge);

    shared_ptr<MultiCostEdge> get(size_t source, size_t target){
        for (auto & ptr: matrix[source]){
            if (ptr->target == target){
                return ptr;
            }
        }

        return nullptr;
    }


    size_t size(void) const{ return this->matrix.size(); };
    size_t get_num_of_objectives() const {return num_of_objectives;};
    // std::vector<shared_ptr<MultiCostEdge>>& operator[](size_t vertex_id);
};



