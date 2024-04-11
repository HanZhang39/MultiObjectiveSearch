#include <cstddef>
#include <memory>
#include <algorithm>
#include <time.h>

#include "BOAStar.h"
#include "SingleCriteria.h"
#include "ShortestPathHeuristic.h"
#include "Utils/Definitions.h"
#include "Utils/pqueue_bucket.h"
#include "Utils/pool.h"



template <class Q>
BOAStar<Q>::BOAStar(AdjacencyMatrix &graph, AdjacencyMatrix & inv_graph,
                    size_t source, size_t target)
    : AbstractSolver(graph, source, target, {0, 0}),
      heuristic(target, inv_graph)
      {
}

template <class Q>
void BOAStar<Q >::solve(unsigned int time_limit) {
    std::vector<cost_t> min_g2(this->graph.size()+1, MAX_COST);

    size_t pool_size = 1024;
    pool<Node<2>> node_pool(pool_size);

    start_time = std::clock();

    NodePtr2 node;
    NodePtr2 next;

    cost_t min_f2 = MAX_COST;
    // Vector to hold mininum cost of 2nd criteria per node

    // Init open heap
    cost_t bucket_width = 1;

    AStar a_star(graph);
    auto sol_top_left = a_star(source, target, heuristic, BO_LEX_ORDER::LEX1);
    cost_t ub1_f = sol_top_left->f[0];

    Q open(bucket_width, heuristic(source)[0], ub1_f);

    node = node_pool.get_label();
    *node = Node2(source, {0,0}, heuristic(source));

    open.push(node);

    while (open.size()) {
        // Pop min from queue and process
        node = open.top();
        open.pop();

        // Dominance check
        if (((node->f[1]) >= min_f2) ||
            (node->f[1] >= min_g2[node->id])) {
            node_pool.save_label(node);
            continue;
        }

        min_g2[node->id] = node->f[1];
        num_expansion += 1;

        if (node->id == target) {
            if (solutions.size() > 0 && node->f[0] == solutions.back().apex[0]){
                solutions.back().apex[1] = node->f[1];
                solutions.back().cost[1] = node->f[1];
            } else {
                vector<cost_t> apex(node->f.begin(), node->f.end());
                solutions.emplace_back(apex);
            }
            // log_solution(node->g);
            min_f2 = node->f[1];
            continue;
        }

        cost_t g0 = node->f[0] - heuristic(node->id)[0];
        cost_t g1 = node->f[1] - heuristic(node->id)[1];

        std::vector<Edge> &outgoing_edges = graph[node->id];
        for (Edge & e:outgoing_edges){
              num_generation +=1;
              size_t next_id = e.target;
              cost_t cost0 = e.cost[0];
              cost_t cost1 = e.cost[1];

              std::array<cost_t, 2> &next_h = heuristic(next_id);
              cost_t f0 = g0 + cost0 + next_h[0];
              cost_t f1 = g1 + cost1 + next_h[1];

              // Dominance check
              if ((f1 >= min_f2) || f0 > ub1_f || (f1 >= min_g2[next_id])) {
                  continue;
              }

              next = node_pool.get_label();
              // next = new Node2(next_id, {f0, f1});
              *next = Node2(next_id, {f0, f1});

              open.push(next);

        }
    }

    std::cout << (double)(std::clock() - start_time) / CLOCKS_PER_SEC << std::endl;
    for (auto& sol: solutions){
        std::cout << sol.cost << std::endl;
    }

}


 


template class BOAStar<pqueue_label_bucket<Node<2>> >;
template class BOAStar<PQ >;
