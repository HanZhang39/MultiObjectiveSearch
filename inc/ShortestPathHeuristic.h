#pragma once

#include "Utils/Definitions.h"
#include "Utils/Graph.h"
#include <cstddef>

template<size_t N>
class ShortestPathHeuristic {
private:
    // size_t                  source;
    // std::vector<NodePtr<N>>    all_nodes;
    std::vector<std::array<cost_t, N>>    data;

    void compute(size_t source, size_t cost_idx, AdjacencyMatrix& adj_matrix, bool reversed);
public:
    ShortestPathHeuristic(size_t source, AdjacencyMatrix &adj_matrix, bool reversed=false);
    inline std::array<cost_t, N> & operator()(size_t node_id){
        // return this->all_nodes[node_id]->f;
        return this->data[node_id];
    }
};

class PartialShortestPathHeuristic {
private:
    size_t                  source;
    size_t num_obj;
    std::vector<cost_t> default_values;

    std::vector<std::unordered_map<size_t, cost_t>> data;

    void set_value(size_t state, size_t cost_idx, size_t value);
    void compute(size_t cost_idx, const AdjacencyMatrix& adj_matrix, size_t goal);
    void compute(size_t cost_idx, const AdjacencyMultiCostMatrix & adj_matrix, size_t goal);
public:
    int node_exp = 0;
    PartialShortestPathHeuristic(size_t source, size_t goal, const AdjacencyMatrix &adj_matrix);
    PartialShortestPathHeuristic(size_t source, size_t goal, const AdjacencyMultiCostMatrix & adj_matrix);
    std::vector<cost_t> operator()(size_t node_id);
};

// biobjective heurisitic with
class BiobjectiveHeuristic {
private:
    // size_t                  source;
    // std::vector<NodePtr<N>>    all_nodes;
    std::vector<std::array<cost_t, 2>>    minc1c2;
    std::vector<std::array<cost_t, 2>>    minc2c1;

    void compute_c1c2(size_t source, AdjacencyMatrix & adj_matrix);
    void compute_c2c1(size_t source, AdjacencyMatrix & adj_matrix);


    void compute(size_t source, size_t cost_idx, AdjacencyMatrix& adj_matrix);
public:
    BiobjectiveHeuristic(size_t source, AdjacencyMatrix &adj_matrix);
    inline std::array<cost_t, 2> operator()(size_t node_id){
        return {this->minc1c2[node_id][0], this->minc2c1[node_id][1]};
    }

    inline std::array<cost_t, 2> get_h_reversed(size_t node_id){
        return {this->minc2c1[node_id][1], this->minc1c2[node_id][0]};
    }

    inline std::array<cost_t, 2> min_c1_cost(size_t node_id){
        return this->minc1c2[node_id];
    }

    inline std::array<cost_t, 2> min_c2_cost(size_t node_id){
        return this->minc2c1[node_id];
    }

};
