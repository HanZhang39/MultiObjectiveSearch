#include <array>
#include <limits>
#include <memory>
#include <algorithm>
#include <unordered_set>

#include "ShortestPathHeuristic.h"
#include "Utils/Graph.h"


template<size_t N>
ShortestPathHeuristic<N>::ShortestPathHeuristic(size_t source, AdjacencyMatrix &adj_matrix, bool reversed)
    : data(adj_matrix.size()) {
    for (auto & entry: data){
        entry.fill(MAX_COST);
    }

    for (int i=0; i < N; i++){
        compute(source, i, adj_matrix, reversed);
    }
}


template<size_t N>
// Implements Dijkstra shortest path algorithm per cost_idx cost function
void ShortestPathHeuristic<N>::compute(size_t source, size_t cost_idx, AdjacencyMatrix &adj_matrix, bool reversed) {
    struct Node{
        size_t idx;
        cost_t value;
        Node() = default;
        Node(size_t idx, cost_t value) : idx(idx), value(value) {}

        struct compare_node {
            // returns true if n1 > n2 (note -- this gives us *min*-heap).
            bool operator()(const Node& n1, const Node& n2) const {
                return n1.value >= n2.value;
            }
        };
    };

    data[source][cost_idx] = 0;
    Node root(source, 0);
    boost::heap::pairing_heap<Node, boost::heap::compare<typename Node::compare_node>> heap;
    size_t num_exp = 0;

    heap.push(root);  // add root to heap
    while (!heap.empty()){
        Node curr = heap.top();
        heap.pop();

        if (data[curr.idx][cost_idx] < curr.value){
            continue;
        }
        num_exp++;

        const std::vector<Edge> &outgoing_edges = adj_matrix[curr.idx];
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            size_t next_state = p_edge->target;

            cost_t next_cost = reversed?
                curr.value + p_edge->cost[N-cost_idx - 1]
                : curr.value+p_edge->cost[cost_idx];
            if (data[next_state][cost_idx] > next_cost){
                data[next_state][cost_idx] = next_cost;
                heap.push({next_state, next_cost});
            }
        }
    }
}

template class ShortestPathHeuristic<2>;
template class ShortestPathHeuristic<3>;
template class ShortestPathHeuristic<4>;
template class ShortestPathHeuristic<5>;
template class ShortestPathHeuristic<6>;


BiobjectiveHeuristic::BiobjectiveHeuristic(size_t source,
                                           AdjacencyMatrix &adj_matrix)
    : minc1c2(adj_matrix.size()),
      minc2c1(adj_matrix.size())
{

    compute_c1c2(source, adj_matrix);
    compute_c2c1(source, adj_matrix);

}


struct NodeLexBiobj{
    size_t idx;
    cost_t value1;
    cost_t value2;
    NodeLexBiobj() = default;
    NodeLexBiobj(size_t idx, cost_t value1, cost_t value2) :
        idx(idx), value1(value1), value2(value2)
    {}

    struct compare_node {
        bool operator()(const NodeLexBiobj& n1, const NodeLexBiobj& n2) const {
            if (n1.value1 != n2.value1){
                return n1.value1 > n2.value1;
            }
            return n1.value2 >= n2.value2;
        }
    };
};



void BiobjectiveHeuristic::compute_c1c2(size_t source, AdjacencyMatrix &adj_matrix) {

    for (auto & entry: minc1c2){
        entry.fill(MAX_COST);
    }

    minc1c2[source] = {0, 0}; // need change
    NodeLexBiobj root(source, 0, 0);
    boost::heap::pairing_heap<NodeLexBiobj, boost::heap::compare<typename NodeLexBiobj::compare_node>> heap;
    size_t num_exp = 0;

    heap.push(root);  // add root to heap
    while (!heap.empty()){
        auto curr = heap.top(); heap.pop();

        if (minc1c2[curr.idx][0] != curr.value1 ||
            minc1c2[curr.idx][1] != curr.value2
            ){
            continue;
        }
        num_exp++;

        const std::vector<Edge> &outgoing_edges = adj_matrix[curr.idx];
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            size_t next_state = p_edge->target;
            cost_t new_cost1 = curr.value1 + p_edge->cost[0];
            cost_t new_cost2 = curr.value2 + p_edge->cost[1];
            if (minc1c2[next_state][0] < new_cost1 ){
                continue;
            }
            if (minc1c2[next_state][0] == new_cost1 &&
                minc1c2[next_state][1] <= new_cost2
                ){
                continue;
            }
            minc1c2[next_state] = {new_cost1, new_cost2};
            heap.push({next_state, new_cost1, new_cost2});
        }
    }
}

void BiobjectiveHeuristic::compute_c2c1(size_t source, AdjacencyMatrix &adj_matrix) {

    for (auto & entry: minc2c1){
        entry.fill(MAX_COST);
    }

    minc2c1[source] = {0, 0}; // need change
    NodeLexBiobj root(source, 0, 0);
    boost::heap::pairing_heap<NodeLexBiobj, boost::heap::compare<typename NodeLexBiobj::compare_node>> heap;
    size_t num_exp = 0;

    heap.push(root);  // add root to heap
    while (!heap.empty()){
        auto curr = heap.top(); heap.pop();

        if (minc2c1[curr.idx][0] != curr.value2 ||
            minc2c1[curr.idx][1] != curr.value1
            ){
            continue;
        }
        num_exp++;

        const std::vector<Edge> &outgoing_edges = adj_matrix[curr.idx];
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            size_t next_state = p_edge->target;
            cost_t new_cost1 = curr.value1 + p_edge->cost[1];
            cost_t new_cost2 = curr.value2 + p_edge->cost[0];
            if (minc2c1[next_state][1] < new_cost1 ){
                continue;
            }
            if (minc2c1[next_state][1] == new_cost1 &&
                minc2c1[next_state][0] <= new_cost2
                ){
                continue;
            }
            minc2c1[next_state] = {new_cost2, new_cost1};
            heap.push({next_state, new_cost1, new_cost2});
        }
    }
}



PartialShortestPathHeuristic::PartialShortestPathHeuristic(size_t source, size_t goal, const AdjacencyMatrix &adj_matrix)
    : source(source)  {
    num_obj = adj_matrix.get_num_of_objectives();

    for (int j=0; j < num_obj; j ++){
        data.push_back(std::unordered_map<size_t,cost_t>());
        compute(j, adj_matrix, goal);
    }
    // assert (default_values.size() == 2);
}




void PartialShortestPathHeuristic::set_value(size_t state, size_t cost_idx, size_t value){
    // if (data[cost_idx].find(state) == data.end()){
    //     data[cost_idx][state] = std::vector<size_t>(num_obj, 0);
    // }

    data[cost_idx][state] = value;
}

// Implements Dijkstra shortest path algorithm per cost_idx cost function
void PartialShortestPathHeuristic::compute(size_t cost_idx, const AdjacencyMatrix &adj_matrix, size_t goal) {
    // Init all heuristics to MAX_COST

    // Init open heap
    std::vector<std::pair<size_t, size_t>> open;
    // std::make_heap(open.begin(), open.end(), more_than);

    // this->all_nodes[this->source]->h[cost_idx] = 0;
    set_value(this->source, cost_idx, 0);
    open.push_back({0, this->source});
    auto order = std::greater<std::pair<size_t, size_t>>();
    std::push_heap(open.begin(), open.end(), order);

    std::unordered_map<size_t, size_t> generated;
    std::unordered_set<size_t> closed;
    size_t previous = 0;

    while (open.empty() == false) {
        // Pop min from queue and process
        std::pop_heap(open.begin(), open.end(), order);
        std::pair<size_t,size_t> curr = open.back();
        open.pop_back();
        size_t cost = curr.first;
        size_t node_id = curr.second;
        assert(cost >= previous);
        previous = cost;

        if (closed.find(node_id) != closed.end()){
            continue;
        }
        node_exp += 1;

        closed.insert(node_id);
        set_value(node_id, cost_idx, cost);

        if (node_id == goal){
            default_values.push_back(cost);
            break;
        }

        // Check to which neighbors we should extend the paths
        const std::vector<Edge> &outgoing_edges = adj_matrix[node_id];
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            // Dominance check
            size_t ch_id = p_edge->target;
            if (generated.find(ch_id)!= generated.end() &&
                generated[ch_id]<= (cost+p_edge->apex[cost_idx])) {
                continue;
            }

            generated[ch_id]= (cost + p_edge->apex[cost_idx]);
            // If not dominated push to queue
            // next->h[cost_idx] = node->h[cost_idx] + p_edge->cost[cost_idx];
            open.push_back({cost + p_edge->apex[cost_idx], ch_id});
            std::push_heap(open.begin(), open.end(), order);
        }
    }
}


std::vector<cost_t> PartialShortestPathHeuristic::operator()(size_t node_id) {
    auto v = default_values;
    for (size_t cost_idx = 0; cost_idx < v.size(); cost_idx++){
        if (data[cost_idx].find(node_id) != data[cost_idx].end()){
            v[cost_idx] = data[cost_idx][node_id];
        }
    }
    return v;
}


// Implements Dijkstra shortest path algorithm per cost_idx cost function
void PartialShortestPathHeuristic::compute(size_t cost_idx, const AdjacencyMultiCostMatrix&adj_matrix, size_t goal) {
    // Init all heuristics to MAX_COST

    // Init open heap
    std::vector<std::pair<size_t, size_t>> open;
    // std::make_heap(open.begin(), open.end(), more_than);

    // this->all_nodes[this->source]->h[cost_idx] = 0;
    set_value(this->source, cost_idx, 0);
    open.push_back({0, this->source});
    auto order = std::greater<std::pair<size_t, size_t>>();
    std::push_heap(open.begin(), open.end(), order);

    std::unordered_map<size_t, size_t> generated;
    std::unordered_set<size_t> closed;
    size_t previous = 0;

    while (open.empty() == false) {
        // Pop min from queue and process
        std::pop_heap(open.begin(), open.end(), order);
        std::pair<size_t,size_t> curr = open.back();
        open.pop_back();
        size_t cost = curr.first;
        size_t node_id = curr.second;
        assert(cost >= previous);
        previous = cost;

        if (closed.find(node_id) != closed.end()){
            continue;
        }
        node_exp += 1;

        closed.insert(node_id);
        set_value(node_id, cost_idx, cost);

        if (node_id == goal){
            default_values.push_back(cost);
            break;
        }

        // Check to which neighbors we should extend the paths
        const auto &outgoing_edges = adj_matrix.inv_matrix[node_id];
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            // Dominance check
            size_t ch_id = (* p_edge )->source;
            if (generated.find(ch_id)!= generated.end() &&
                generated[ch_id]<= (cost+(* p_edge )->min_cost[cost_idx])) {
                continue;
            }

            generated[ch_id]= (cost + (* p_edge )->min_cost[cost_idx]);
            // If not dominated push to queue
            // next->h[cost_idx] = node->h[cost_idx] + p_edge->cost[cost_idx];
            open.push_back({cost +(* p_edge )->min_cost[cost_idx], ch_id});
            std::push_heap(open.begin(), open.end(), order);
        }
    }
}



PartialShortestPathHeuristic::PartialShortestPathHeuristic(size_t source, size_t goal, const AdjacencyMultiCostMatrix &adj_matrix)
    : source(source)  {
    num_obj = adj_matrix.get_num_of_objectives();

    for (int j=0; j < num_obj; j ++){
        data.push_back(std::unordered_map<size_t,cost_t>());
        compute(j, adj_matrix, goal);
    }
    // assert (default_values.size() == 2);
}
