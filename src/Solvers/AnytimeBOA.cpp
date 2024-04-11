#include "Utils/Definitions.h"
#include "Utils/common.h"
#include <AnytimeBOA.h>
#include "SingleCriteria.h"

Interval::Interval(const NodePtr2 top_left, const NodePtr2 bottom_right, std::shared_ptr<std::list<NodePtr2>> to_expand): top_left(top_left), bottom_right(bottom_right), to_expand(to_expand){
  eps = 0;
  //this->to_expand.reserve(to_expand.size());
  for (auto& node: *to_expand){
    eps = std::max(eps, std::min( ((double)top_left->f[1]) / node->f[1] - 1, ((double)bottom_right->f[0])/node->f[0] - 1  ));
  }
}

std::ostream& operator<<(std::ostream& os, const Interval& interval){
  os << "Top left: " << interval.top_left->f  << ", Bottom right: " << interval.bottom_right->f << ", #nodes: " << interval.to_expand->size();
  return os;
}

void BOAStarContinuing::solve(Interval source, IntervalList & solutions, unsigned int time_limit){

    auto start_time = std::clock();
    std::vector<size_t> min_g2(this->graph.size()+1, MAX_COST);


    size_t pool_size = 1024;
    pool<Node<2>> node_pool(pool_size);

    NodePtr2 node;
    NodePtr2 next;

    solutions.clear();

    min_g2[target] = source.top_left->f[1];
    size_t max_f1 = source.bottom_right->f[0];

    // Init open heap
    // heap_open_t open;
    std::vector<NodePtr2> open;
    Node<2>::more_than_full_cost more_than;
    std::shared_ptr<std::list<NodePtr2>> not_expanded = std::make_shared<std::list<NodePtr2>>();
    NodePtr2 prev = source.top_left;

    for (auto & node:*source.to_expand){
        open.push_back(node);
    }
    std::make_heap(open.begin(), open.end(), more_than);

    bool flag = false;
    while (open.empty() == false) {
        if ((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){
            Interval intval(prev, source.bottom_right, not_expanded);
            solutions.push_back(intval);
            return;
        }

        // Pop min from queue and process
        std::pop_heap(open.begin(), open.end(), more_than);
        node = open.back();
        open.pop_back();
        num_generation +=1;

        // Dominance check
        if (node->f[0] >= max_f1){
            continue;
        }

        cost_t g0 = node->f[0] - heuristic(node->id)[0];
        cost_t g1 = node->f[1] - heuristic(node->id)[1];

        if ((((1+this->eps[1])*node->f[1]) >= min_g2[target]) ||
            (g1 >= min_g2[node->id])) {
            if (g1 < min_g2[node->id] && node->f[1] < min_g2[target]){
                not_expanded->push_back(node);
                min_g2[node->id] = g1;
            } else {
                node_pool.save_label(node);
                continue;
            }
        }

        min_g2[node->id] = g1;
        num_expansion += 1;

        if (node->id == target) {
            // log_solution(node);
            Interval intval(prev, node, not_expanded);
            solutions.push_back(intval);
            prev = node;
            not_expanded = std::make_shared<std::list<NodePtr2>>() ;
            continue;
        }

        // Check to which neighbors we should extend the paths
        const std::vector<Edge> &outgoing_edges = graph[node->id];
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            size_t next_id = p_edge->target;
            std::vector<cost_t> next_g = {g0+p_edge->cost[0], g1+p_edge->cost[1]};
            auto next_h = heuristic(next_id);

            // Dominance check
            if (next_g[0]+next_h[0] >= max_f1){
                continue;
            }
            // if (l_heuristics != nullptr){
            //     flag = false;
            //     for (int i = 0; i < l_heuristics->size(); i++){
            //         // assert(l_heuristics->at(i)(next_id) == ll_heuristics->at(i)(next_id));

            //         auto h_val = (*(l_heuristics->at(i)))(next_id);
            //         if (next_g[0] + h_val - l_heuristics->at(i)->get_factor() * (min_g2[target] - next_g[1]) >= max_f1 ){
            //     // for (auto & lch: *l_heuristics){
            //     //     if (next_g[0] + lch(next_id) - lch.get_factor() * (min_g2[target] - next_g[1]) >= max_f1 ){
            //             flag = true;
            //             break;
            //         }
            //     }
            //     if (flag){
            //         continue;
            //     }
            // }


            if ((((1+this->eps[1])*(next_g[1]+next_h[1])) >= min_g2[target]) ||
                (next_g[1] >= min_g2[next_id])) {
                if (next_g[1] < min_g2[next_id] && (next_g[1]+next_h[1]) < min_g2[target]){
                    next = node_pool.get_label();
                    *next = Node2(next_id, {next_g[0] + next_h[0], next_g[1] + next_h[1]});
                    not_expanded->push_back(next);
                }
                continue;
            }

            // If not dominated create node and push to queue
            // Creation is defered after dominance check as it is
            // relatively computational heavy and should be avoided if possible
            next = node_pool.get_label();
            *next = Node2(next_id, {next_g[0] + next_h[0], next_g[1] + next_h[1]});

            open.push_back(next);
            std::push_heap(open.begin(), open.end(), more_than);

        }
    }

    Interval intval(prev, source.bottom_right, not_expanded);
    solutions.push_back(intval);

}


void AnytimeBOA::solve(Interval source, IntervalList & solutions, double eps, unsigned int time_limit){

    // auto start_time = std::clock();
    std::vector<size_t> min_g2(this->graph.size()+1, MAX_COST);

    NodePtr2 node;
    NodePtr2 next;

    solutions.clear();

    min_g2[target] = source.top_left->f[1];
    size_t max_f1 = source.bottom_right->f[0];

    // Init open heap
    // heap_open_t open;
    std::vector<NodePtr2> open;
    Node<2>::more_than_full_cost more_than;
    std::shared_ptr<std::list<NodePtr2>> not_expanded = std::make_shared<std::list<NodePtr2>>();
    NodePtr2 prev = source.top_left;

    for (auto & node:*source.to_expand){
        open.push_back(node);
    }
    std::make_heap(open.begin(), open.end(), more_than);

    bool flag = false;
    while (open.empty() == false) {
        if ((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){
            Interval intval(prev, source.bottom_right, not_expanded);
            solutions.push_back(intval);
            return;
        }

        // Pop min from queue and process
        std::pop_heap(open.begin(), open.end(), more_than);
        node = open.back();
        open.pop_back();
        num_generation +=1;

        // Dominance check
        if (node->f[0] >= max_f1){
            continue;
        }

        cost_t g0 = node->f[0] - heuristic(node->id)[0];
        cost_t g1 = node->f[1] - heuristic(node->id)[1];

        if ((((1+eps)*node->f[1]) >= min_g2[target]) ||
            (g1 >= min_g2[node->id])) {
            if (g1 < min_g2[node->id] && node->f[1] < min_g2[target]){
                not_expanded->push_back(node);
                min_g2[node->id] = g1;
            } else {
                node_pool.save_label(node);
                continue;
            }
        }

        min_g2[node->id] = g1;
        num_expansion += 1;

        if (node->id == target) {
            log_solution(node);
            Interval intval(prev, node, not_expanded);
            solutions.push_back(intval);
            prev = node;
            not_expanded = std::make_shared<std::list<NodePtr2>>() ;
            continue;
        }

        // Check to which neighbors we should extend the paths
        const std::vector<Edge> &outgoing_edges = graph[node->id];
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            size_t next_id = p_edge->target;
            std::vector<cost_t> next_g = {g0+p_edge->cost[0], g1+p_edge->cost[1]};
            auto next_h = heuristic(next_id);

            // Dominance check
            if (next_g[0]+next_h[0] >= max_f1){
                continue;
            }
            // if (l_heuristics != nullptr){
            //     flag = false;
            //     for (int i = 0; i < l_heuristics->size(); i++){
            //         // assert(l_heuristics->at(i)(next_id) == ll_heuristics->at(i)(next_id));

            //         auto h_val = (*(l_heuristics->at(i)))(next_id);
            //         if (next_g[0] + h_val - l_heuristics->at(i)->get_factor() * (min_g2[target] - next_g[1]) >= max_f1 ){
            //     // for (auto & lch: *l_heuristics){
            //     //     if (next_g[0] + lch(next_id) - lch.get_factor() * (min_g2[target] - next_g[1]) >= max_f1 ){
            //             flag = true;
            //             break;
            //         }
            //     }
            //     if (flag){
            //         continue;
            //     }
            // }


            if ((((1+eps)*(next_g[1]+next_h[1])) >= min_g2[target]) ||
                (next_g[1] >= min_g2[next_id])) {
                if (next_g[1] < min_g2[next_id] && (next_g[1]+next_h[1]) < min_g2[target]){
                    next = node_pool.get_label();
                    *next = Node2(next_id, {next_g[0] + next_h[0], next_g[1] + next_h[1]});
                    not_expanded->push_back(next);
                }
                continue;
            }

            // If not dominated create node and push to queue
            // Creation is defered after dominance check as it is
            // relatively computational heavy and should be avoided if possible
            next = node_pool.get_label();
            *next = Node2(next_id, {next_g[0] + next_h[0], next_g[1] + next_h[1]});

            open.push_back(next);
            std::push_heap(open.begin(), open.end(), more_than);

        }
    }

    Interval intval(prev, source.bottom_right, not_expanded);
    solutions.push_back(intval);

}



struct interval_comp{
    bool operator()(const Interval n1, const Interval n2) const
    {
        return n1.eps < n2.eps;
    }
};
typedef boost::heap::pairing_heap<Interval, boost::heap::compare<interval_comp> > IntervalQueue;


void AnytimeBOA::solve(unsigned int time_limit){
    start_time = std::clock();

    AStar a_star(graph);
    auto sol_top_left = a_star(source, target, heuristic, BO_LEX_ORDER::LEX1);
    auto sol_bottom_right = a_star(source, target, heuristic, BO_LEX_ORDER::LEX0);
    log_solution(sol_bottom_right);
    log_solution(sol_top_left);

    clock_t boa_time = 0;

    auto node = node_pool.get_label();

    *node = Node<2>(source, {0,0}, heuristic(source));

    auto tmp_ptr =std::make_shared<std::list<NodePtr2>>();
    tmp_ptr->push_back(node);
    IntervalQueue to_expand;
    to_expand.push(Interval(sol_bottom_right, sol_top_left, tmp_ptr));

    while(true){
        if ((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){
            solutions.clear();
            solutions.reserve(to_expand.size() + 1);
            NodePtr2 rightmost=nullptr;
            for (auto interval: to_expand){
                if (rightmost == nullptr){
                    rightmost = interval.bottom_right;
                } else if(interval.bottom_right->f[0] > rightmost->f[0]){
                    rightmost = interval.bottom_right;
                }
                // solutions.push_back(interval.top_left);
            }
            // solutions.push_back(rightmost);
            // log_all_solutions();

            return;
        }
 

        auto interval =  to_expand.top();
        if (interval.eps == 0){
            // Found all solutions;
            // solutions.clear();
            // solutions.reserve(to_expand.size() + 1);
            NodePtr2 rightmost=nullptr;
            for (auto interval: to_expand){
                if (rightmost == nullptr){
                    rightmost = interval.bottom_right;
                } else if(interval.bottom_right->f[0] > rightmost->f[0]){
                    rightmost = interval.bottom_right;
                }
                // solutions.push_back(interval.top_left);
            }
            // solutions.push_back(rightmost);
            std::cout << "BOA conti time: " << boa_time/CLOCKS_PER_SEC << " s" << std::endl;
            // log_all_solutions();

            // std::clock_t lazy_h_compute_time = 0;
            // for (auto it_lh: *lh){
            //     lazy_h_compute_time += it_lh->heuristic_time;
            // }
            // std::cout << "  lazy h compute time " << ((double)lazy_h_compute_time)/CLOCKS_PER_SEC<< std::endl;


            return;
        }
        to_expand.pop();
        std::cout << "  interval eps " << interval.eps<< std::endl;

        IntervalList refined_interval;
        // auto boa_start = std::clock();
        // BOAStarContinuing boac(adj_matrix, interval.eps / d);
        // boac.set_parent(this);
        // boac(interval, target, heuristic, refined_interval, lh, time_limit -(std::clock() - start_time)/CLOCKS_PER_SEC + 1);
        // boa_time += std::clock() - boa_start;

        solve(interval, refined_interval, interval.eps/d, time_limit);

        // solution_log.insert(solution_log.end(), boac.solution_log.begin(), boac.solution_log.end());
        // num_expansion += boac.get_num_expansion();
        // num_generation += boac.get_num_generation();

        for (auto new_interval:refined_interval){
            std::cout << "     new interval eps " << new_interval.eps<< std::endl;
            to_expand.push(new_interval);
        }

        if ((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){
            continue;
        }

    }
    // log_all_solutions();



}


void AnytimeBOA::log_solution(NodePtr2 node){
    vector<cost_t> apex(node->f.begin(), node->f.end());
    this->solutions.emplace_back(apex, apex, clock() - this->start_time);
}
