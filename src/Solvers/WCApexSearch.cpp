#include "WCApexSearch.h"
#include "AbstractSolver.h"
#include "GCL.h"
#include "Utils/Definitions.h"
#include "SingleCriteria.h"

#include "Utils/pool.h"
#include "Utils/pqueue_bucket.h"
#include <array>
#include <iostream>

using namespace std::chrono_literals;


bool update_nodes_by_merge_rcapex(const ApexPathPairPtr<2> & ap, const ApexPathPairPtr<2> &other, CostVec<2> h, const std::vector<double> eps){
    
    if (ap->id != other->id) {
        return false;
    }

    CostVec<2> new_apex = min<2>(ap->f, other->f );
    CostVec<2> * new_path_cost_ptr= nullptr;

    if (ap->path_cost[1] != other->path_cost[1]){
        new_path_cost_ptr = ap->path_cost[1] < other->path_cost[1]? &ap->path_cost : &other->path_cost;
    } else {
        new_path_cost_ptr = ap->path_cost[0] < other->path_cost[0]? &ap->path_cost : &other->path_cost;
    }

    if (! is_bounded<2>(new_apex, * new_path_cost_ptr, h, eps)){
        return false;
    }

    ap->f = new_apex;
    ap->path_cost = *new_path_cost_ptr;
    return true;
}

WCApexSearch::WCApexSearch(AdjacencyMatrix & graph, AdjacencyMatrix & inv_graph,
                           size_t source,
                           size_t target, int bound_ratio, double eps)
    : ApexSearch<2, G2min>(graph, inv_graph, source, target, {eps, 0}), bound_ratio(bound_ratio),
      inv_graph(inv_graph),
      bi_heuristic(target, inv_graph),
      inv_bi_heuristic(source, graph) ,
      open_map_bwd(graph.size(), nullptr)

       {

    AStar a_star(graph);
    auto sol_bottom_right = a_star(source, target, heuristic, BO_LEX_ORDER::LEX1);
    auto sol_top_left = a_star(source, target, heuristic, BO_LEX_ORDER::LEX0);
    lb2_f = sol_bottom_right->f[1];
    ub2_f = sol_top_left->f[1];
    lb1_f = sol_top_left->f[0];
    ub1_f = sol_bottom_right->f[0];

    resource_bound = lb2_f + (ub2_f - lb2_f) * bound_ratio / 100;

}

void WCApexSearch::solve(unsigned int time_limit){
    // solve_backword(time_limit);
    // return;

    init_search();
    pool<ApexPathPair<2>> ap_pool = pool<ApexPathPair<2>>(POOL_SIZE);

    ApexPathPairPtr<2> ap = nullptr;
    ApexPathPairPtr<2> next_ap = nullptr;

    std::cout << source << ", " << target << std::endl;


    cost_t bucket_width = 1;
    pqueue_label_bucket<ApexPathPair<2>> open(bucket_width, heuristic(source)[0],
                                              ub1_f);

    ap = ap_pool.get_label();

    CostVec<2> zero_vec;
    zero_vec.fill(0);

    // std::cout << "initial heuristic" << heuristic(source)[0] << ", " << heuristic(source)[1] << std::endl;


    *ap = ApexPathPair<2>(source, heuristic(source), zero_vec);

    open.push(ap);
    open_map.add(ap);

    std::vector<cost_t> min_f2(this->graph.size() + 1, MAX_COST);

    auto start = std::chrono::steady_clock::now(); 
    start_time_ = std::chrono::steady_clock::now(); 
    // (cost, resource)
    while (open.size()) {
        // Pop min from queue and process
        ap = open.top();
        open.pop();

        if (!ap->active){
            ap_pool.save_label(ap);
            continue;
        }

        open_map.remove(ap);

        // Dominance check
        if (ap->f[1] >= min_f2[ap->id] or
            (1 + eps[0]) * ap->f[0] >= best_solution_so_far[0]
            ) {
            ap_pool.save_label(ap);
            continue;
        }

        min_f2[ap->id] = ap->f[1];

        num_expansion += 1;

        if (ap->id == target) {
            // this->merge_to_solutions(ap, ap_solutions);
            // TODO merge to solution
            // ap_solutions.push_back(ap);
            std::cout << "reach target" << std::endl ;
            break;
        }

        cost_t g0 = ap->f[0] - heuristic(ap->id)[0];
        cost_t g1 = ap->f[1] - heuristic(ap->id)[1];

        // Check to which neighbors we should extend the paths
        const std::vector<Edge> &outgoing_edges = graph[ap->id];
        for (const Edge & e:outgoing_edges){
            num_generation +=1;
            size_t next_id = e.target;
            cost_t cost0 = e.cost[0];
            cost_t cost1 = e.cost[1];
            // next_ap = extend(ap, *p_edge, heuristic);

            auto next_h = heuristic(next_id);

            cost_t f0 = g0 + cost0 + next_h[0];
            cost_t f1 = g1 + cost1 + next_h[1];
            if (f1 >= min_f2[next_id] or f0 > ub1_f or f1 > resource_bound) {
                continue;
            }

            next_ap = ap_pool.get_label();
            // next = new Node2(next_id, {f0, f1});
            *next_ap = ApexPathPair<2>(next_id,
                                       {f0, f1},
                                       sum<2>(ap->path_cost, {cost0, cost1}),
                                       ap);
 

            if (next_ap->path_cost[1] + bi_heuristic.min_c1_cost(next_id)[1] <= resource_bound
                && next_ap->path_cost[0] + bi_heuristic.min_c1_cost(next_id)[0] < best_solution_so_far[0]
                ){
                best_solution_so_far = {next_ap->path_cost[0] + bi_heuristic.min_c1_cost(next_id)[0],
                                        next_ap->path_cost[1] + bi_heuristic.min_c1_cost(next_id)[1]
                };
            }
            if (next_ap->path_cost[1] + bi_heuristic.min_c2_cost(next_id)[1] <= resource_bound
                && next_ap->path_cost[0] + bi_heuristic.min_c2_cost(next_id)[0] < best_solution_so_far[0]
                ){
                best_solution_so_far = {next_ap->path_cost[0] + bi_heuristic.min_c2_cost(next_id)[0],
                                        next_ap->path_cost[1] + bi_heuristic.min_c2_cost(next_id)[1]
                };
            }


            auto ptr = open_map[next_id];

            while (ptr!= nullptr){
                if (update_nodes_by_merge_rcapex(next_ap, ptr, heuristic(next_id), eps)){
                    open_map.remove(ptr);
                    ptr->active = false;
                    break; // do we really need to break?
                }
                ptr = ptr->next_in_map_;
            }


            open_map.add(next_ap);
            open.push(next_ap);
            // this->insert(next_ap);
        }

        ap_pool.save_label(ap);
    }

    solutions.push_back(Solution({best_solution_so_far[0], best_solution_so_far[1]}));

    auto end = std::chrono::steady_clock::now(); 
    printf("Work took %f seconds\n", (end - start)/1.0s  );
}


void add_to_openmap_bwd(std::vector<ApexPathPair<2>*> &open_map_bwd, ApexPathPairPtr<2> ap){
    if (open_map_bwd[ap->id] != nullptr){
        open_map_bwd[ap->id] -> prev_in_map_ = ap;
    }
    ap->next_in_map_ = open_map_bwd[ap->id];
    open_map_bwd[ap->id] = ap;
}

void remove_from_openmap_bwd(std::vector<ApexPathPair<2>*> &open_map_bwd, ApexPathPairPtr<2> ap) {
    if (open_map_bwd[ap->id] == ap){
        if (ap->next_in_map_ != nullptr){
            ap->next_in_map_->prev_in_map_ = nullptr;
        }
        open_map_bwd[ap->id] = ap->next_in_map_;
        ap->next_in_map_ = nullptr;
    } else {
        ap->prev_in_map_->next_in_map_ = ap->next_in_map_;
        if (ap->next_in_map_!= nullptr){
            ap->next_in_map_->prev_in_map_ = ap->prev_in_map_;
        }
        ap->prev_in_map_ = nullptr;
        ap->next_in_map_ = nullptr;
    }

}

void WCApexSearch::solve_backword(unsigned int time_limit){

    pool<ApexPathPair<2>> ap_pool = pool<ApexPathPair<2>>(POOL_SIZE);
    EPS inv_eps = {eps[1], eps[0]};

    ApexPathPairPtr<2> ap = nullptr;
    ApexPathPairPtr<2> next_ap = nullptr;

    std::cout << source << ", " << target << std::endl;

    cost_t bucket_width = 1;
    pqueue_label_bucket<ApexPathPair<2>> open(bucket_width, heuristic(source)[1], resource_bound);

    ap = ap_pool.get_label();
    *ap = ApexPathPair<2>(target, inv_bi_heuristic.get_h_reversed(target), {0, 0});

    open.push(ap);
    add_to_openmap_bwd(open_map_bwd, ap);

    std::vector<cost_t> min_f2(this->graph.size() + 1, MAX_COST);

    while (open.size()) {

        // resource increase, cost decrease
        ap = open.top();
        open.pop();

        if (!ap->active){
            ap_pool.save_label(ap);
            continue;
        }

        remove_from_openmap_bwd(open_map_bwd, ap);

        // Dominance check
        if (ap->f[1] >= min_f2[ap->id] or
             (1 + eps[0]) * ap->f[1] >= best_solution_so_far[0]
            ) {
            ap_pool.save_label(ap);
            continue;
        }



        min_f2[ap->id] = ap->f[1];

        num_expansion_bwd += 1;

        if (ap->id == source) {
            // best_solution_so_far = ap;
            continue;
        }

        cost_t g0 = ap->f[0] - inv_bi_heuristic(ap->id)[1];
        cost_t g1 = ap->f[1] - inv_bi_heuristic(ap->id)[0];

        // Check to which neighbors we should extend the paths
        const std::vector<Edge> &outgoing_edges = inv_graph[ap->id];
        for (const Edge & e:outgoing_edges){
            num_generation_bwd +=1;
            size_t next_id = e.target;
            cost_t cost0 = e.cost[1];
            cost_t cost1 = e.cost[0];

            auto next_h = inv_bi_heuristic(next_id); // note that this h-value is inversed 

            cost_t f0 = g0 + cost0 + next_h[1];
            cost_t f1 = g1 + cost1 + next_h[0];
            if (f1 >= min_f2[next_id] or
                f0 > resource_bound or f1 > ub1_f or
                 (1 + eps[0]) * f1 >= best_solution_so_far[0] ){
                continue;
            }

            next_ap = ap_pool.get_label();
            *next_ap = ApexPathPair<2>(next_id, {f0, f1}, sum<2>(ap->path_cost, {cost0, cost1}), ap);

            // (resource, cost)
            if (next_ap->path_cost[0] + inv_bi_heuristic.min_c1_cost(next_id)[1] <= resource_bound
                && next_ap->path_cost[1] + inv_bi_heuristic.min_c1_cost(next_id)[0] < best_solution_so_far[0]
                ){
                best_solution_so_far = {next_ap->path_cost[1] + inv_bi_heuristic.min_c1_cost(next_id)[0],
                                        next_ap->path_cost[0] + inv_bi_heuristic.min_c1_cost(next_id)[1]
                };
            }

            if (next_ap->path_cost[0] + inv_bi_heuristic.min_c2_cost(next_id)[1] <= resource_bound
                && next_ap->path_cost[1] + inv_bi_heuristic.min_c2_cost(next_id)[0] < best_solution_so_far[0]
                ){
                best_solution_so_far = {next_ap->path_cost[1] + inv_bi_heuristic.min_c2_cost(next_id)[0],
                                        next_ap->path_cost[0] + inv_bi_heuristic.min_c2_cost(next_id)[1]
                };
            }

            auto ptr = open_map_bwd[next_id];

            while (ptr!= nullptr){
                if (update_nodes_by_merge_if_bounded<2>(next_ap, ptr, inv_bi_heuristic.get_h_reversed(next_id), inv_eps, ms)){
                    remove_from_openmap_bwd(open_map_bwd, ptr);
                    ptr->active = false;
                    break; // do we really need to break?
                }
                ptr = ptr->next_in_map_;
            }


            add_to_openmap_bwd(open_map_bwd, next_ap);
            open.push(next_ap);
        }
    }

    solutions.push_back(Solution({best_solution_so_far[0], best_solution_so_far[1]}));
}
