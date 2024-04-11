#include "SingleCriteria.h"
#include "ShortestPathHeuristic.h"
#include "Utils/Definitions.h"
#include <cstddef>
#include <unordered_set>

AStar::AStar(const AdjacencyMatrix &adj_matrix):
    adj_matrix(adj_matrix) {};

struct BOLexOrder {
    BO_LEX_ORDER order;
    BOLexOrder(BO_LEX_ORDER order): order(order){}

  bool operator()(const NodePtr<2> &a, const NodePtr<2> &b) const {
    if (order == BO_LEX_ORDER::LEX0) {
      if (a->f[0] != b->f[0]) {
        return (a->f[0] > b->f[0]);
      } else {
        return (a->f[1] > b->f[1]);
      }
    } else {
      if (a->f[1] != b->f[1]) {
        return (a->f[1] > b->f[1]);
      } else {
        return (a->f[0] > b->f[0]);
      }
    }
  }
};

NodePtr<2> AStar::operator()(size_t source, size_t target, ShortestPathHeuristic<2> &heuristic, BO_LEX_ORDER order) {

    // Saving all the unused NodePtrs in a vector improves performace for some reason
    std::unordered_set<size_t> closed;

    // Init open heap
    BOLexOrder more_than(order);
    std::vector<NodePtr<2>> open;
    std::make_heap(open.begin(), open.end(), more_than);

    auto node = new Node<2>(source, heuristic(source));
    open.push_back(node);
    std::push_heap(open.begin(), open.end(), more_than);

    while (open.empty() == false) {
        // Pop min from queue and process
        std::pop_heap(open.begin(), open.end(), more_than);
        node = open.back();
        open.pop_back();

        if (closed.find(node->id) != closed.end()){
            continue;
        }
        closed.insert(node->id);

        // Dominance check
        if (node->id == target) {
            return node;
        }
        cost_t g0 = node->f[0] - heuristic(node->id)[0];
        cost_t g1 = node->f[1] - heuristic(node->id)[1];

        // Check to which neighbors we should extend the paths
        const std::vector<Edge> &outgoing_edges = adj_matrix[node->id];
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            size_t next_id = p_edge->target;
            auto next_h = heuristic(next_id);
            std::array<cost_t, 2> next_f = {g0 + p_edge->cost[0] + next_h[0],
                                            g1 + p_edge->cost[1] + next_h[1]};

            if (closed.find(next_id) != closed.end()){
                continue;
            }

            // If not dominated create node and push to queue
            // Creation is defered after dominance check as it is
            // relatively computational heavy and should be avoided if possible
            auto next = new Node<2>(next_id, next_f);

            open.push_back(next);
            std::push_heap(open.begin(), open.end(), more_than);

            // closed.push_back(node);
        }
    }

}
