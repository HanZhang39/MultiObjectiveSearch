#include <cstddef>
#include <ctime>
#include <deque>
#include <memory>
#include <queue>
#include <sys/_types/_clock_t.h>
#include <vector>

#include <iostream>

#include "ApexSearch.h"
#include "GCL.h"
#include "SingleCriteria.h"
#include "ShortestPathHeuristic.h"
#include "SingleCriteria.h"

#include "Utils/Definitions.h"
#include "Utils/pool.h"
#include "Utils/pqueue_bucket.h"


template<int N>
bool is_bounded(CostVec<N> & apex, CostVec<N> & pathcost, CostVec<N> & h, const EPS eps){
    for (int i = 0; i < N; i ++ ){
        if (pathcost[i] + h[i] > (cost_t) ((1 + eps[i]) * (apex[i]))){
            return false;
        }
    }
    return true;
}

template bool is_bounded<2>(CostVec<2> & apex, CostVec<2> & pathcost, CostVec<2> & h, const EPS eps);
template bool is_bounded<3>(CostVec<3> & apex, CostVec<3> & pathcost, CostVec<3> & h, const EPS eps);
template bool is_bounded<4>(CostVec<4> & apex, CostVec<4> & pathcost, CostVec<4> & h, const EPS eps);
template bool is_bounded<5>(CostVec<5> & apex, CostVec<5> & pathcost, CostVec<5> & h, const EPS eps);
template bool is_bounded<6>(CostVec<6> & apex, CostVec<6> & pathcost, CostVec<6> & h, const EPS eps);



template <int N>
double compute_slack(CostVec<N> &apex, CostVec<N> &path_cost, CostVec<N> & h, 
                     const EPS eps) {
    double min_slack =
        ((1 + eps[0]) - (double) (path_cost[0] +h[0] )/ (double)apex[0]) / eps[0];
    // ((double) (path_cost[0] +h[0] )/ (double)apex[0]) / (1.0 + eps[0]);
    for (int i = 1; i < N; i++) {
        double slack =
            ((1 + eps[i]) - (double) (path_cost[i] +h[i]) / (double)apex[i]) / eps[i];
        // ((double) (path_cost[i] +h[i] )/ (double)apex[i]) / (1.0 + eps[i]);
        if (slack < min_slack) {
            min_slack = slack;
        }
        // min_slack += slack;
    }
    return min_slack;
}

template double compute_slack<2>(CostVec<2> &apex, CostVec<2> &path_cost, CostVec<2> &h, const EPS eps) ;
template double compute_slack<3>(CostVec<3> &apex, CostVec<3> &path_cost, CostVec<3> &h, const EPS eps) ;
template double compute_slack<4>(CostVec<4> &apex, CostVec<4> &path_cost, CostVec<4> &h, const EPS eps) ;
template double compute_slack<5>(CostVec<5> &apex, CostVec<5> &path_cost, CostVec<5> &h, const EPS eps) ;
template double compute_slack<6>(CostVec<6> &apex, CostVec<6> &path_cost, CostVec<6> &h, const EPS eps) ;


template< int N>
double delta_slack(const ApexPathPairPtr<N> & ap, const ApexPathPairPtr<N> &other, CostVec<N> h, const std::vector<double> eps){
    CostVec<N> new_apex = min<N>(ap->f, other->f );
    return max(compute_slack<N>(new_apex, ap->path_cost, h, eps),
               compute_slack<N>(new_apex, other->path_cost, h, eps)
               );
}

template< int N>
bool update_nodes_by_merge_if_bounded(const ApexPathPairPtr<N> & ap, const ApexPathPairPtr<N> &other, CostVec<N> h, const std::vector<double> eps, MergeStrategy s){
    // Returns true on sucessful merge and false if it failure
    if (ap->id != other->id) {
        return false;
    }

    CostVec<N> new_apex = min<N>(ap->f, other->f );
    CostVec<N> * new_path_cost_ptr= nullptr;

    // choose a path node
    if (s == MergeStrategy::SMALLER_G2 || s == MergeStrategy::SMALLER_G2_FIRST){
        std::cerr << "not implemented " << std::endl;
        exit(-1);
    }else if (s == MergeStrategy::RANDOM){

        if (is_bounded<N>(new_apex, ap->path_cost, h, eps)){
            new_path_cost_ptr= & ap->path_cost;
        }
        if ( (new_path_cost_ptr == nullptr || rand()%2 ==1) &&
             is_bounded<N>(new_apex, other->path_cost, h, eps)){
            new_path_cost_ptr= & ap->path_cost;
        }
        if (new_path_cost_ptr == nullptr){
            return false;
        }
    }else if (s == MergeStrategy::MORE_SLACK){
        if (is_bounded<N>(new_apex, ap->path_cost, h, eps)){
            new_path_cost_ptr= & ap->path_cost;
        }
        if (is_bounded<N>(new_apex, other->path_cost, h, eps)){
            if (new_path_cost_ptr== nullptr){
                new_path_cost_ptr = & other->path_cost;
            }else if ( compute_slack<N>(new_apex, other->path_cost, h, eps) > compute_slack<N>(new_apex, * new_path_cost_ptr, h, eps)){
                new_path_cost_ptr = & other->path_cost;
            }
        }
        if (new_path_cost_ptr == nullptr){
            return false;
        }
    }else if (s == MergeStrategy::REVERSE_LEX){
        new_path_cost_ptr = & ap->path_cost;
        // CostVec<N> * other_path_cost_ptr = & ap->path_cost;
        for (int i = new_apex.size() - 1; i >= 0; i--){
            if (ap->path_cost[i] != other->path_cost[i]){
                new_path_cost_ptr = ap->path_cost[i] < other->path_cost[i] ? &ap->path_cost: &other->path_cost;
                // other_path_cost_ptr = ap->path_cost[i] >= other->path_cost[i] ? &ap->path_cost: &other->path_cost;
                break;
            }
        }
        if (! is_bounded<N>(new_apex, * new_path_cost_ptr, h, eps)){
            return false;
            // new_path_cost_ptr = other_path_cost_ptr;
            // if (! is_bounded<N>(new_apex, * new_path_cost_ptr, h, eps)){
            //     return false;
            // }
        }
    }else{
        std::cerr << "merge strategy not known" << std::endl;
        exit(-1);
    }

    ap->f = new_apex;
    ap->parent = new_path_cost_ptr == & ap->path_cost ? ap->parent : other->parent;
    if (new_path_cost_ptr != &ap->path_cost){
        CostVec<N> tmp_path_cost = ap->path_cost;
        ap->path_cost = *new_path_cost_ptr;
        // *new_path_cost_ptr = tmp_path_cost;
        other->path_cost = tmp_path_cost;
    }
    return true;
}

template <int N>
bool update_apex(const ApexPathPairPtr<N>& ap, const ApexPathPairPtr<N>& other, const std::array<double, N> eps){
    // if (!is_bounded(other_apex, path_node, eps)){
    //   return false;
    // }
    std::vector<size_t> new_apex = ap->g;
    bool update_flag = false;
    // Merge apex
    for (int i = 0; i < other->g.size(); i ++){
        if (other->f[i] < new_apex[i]){
            new_apex[i] = other->f[i];
            if ( ap->path_cost[i] > (1 + eps[i]) * new_apex[i] ){
                return false;
            }

            update_flag = true;
        }
    }
    if (update_flag){
        ap->g = new_apex;
        ap->f = new_apex;
    }
    return true;
}

template <int N, class GCL>
ApexSearch<N, GCL>::ApexSearch(AdjacencyMatrix & graph, AdjacencyMatrix & inv_graph,
                          size_t source, size_t target,
                          EPS eps)
    : AbstractSolver(graph, source, target, eps),
      num_of_objectives(graph.get_num_of_objectives()),
      gcl_ptr(std::make_unique<GCL>(graph.size())),
      heuristic(ShortestPathHeuristic<N>(target, inv_graph)),
      open_map(graph.size())
      {
}

template <int N, class GCL>
void ApexSearch<N, GCL>::solve(unsigned int time_limit) {

    init_search();
    // pool<ApexPathPair<N>> ap_pool = pool<ApexPathPair<N>>(POOL_SIZE);

    ApexPathPairPtr<N> ap = nullptr;
    ApexPathPairPtr<N> next_ap = nullptr;

    start_time = std::clock();

    // Only work for 2 objectives
    // AStar a_star(graph);
    // using std::placeholders::_1;
    // auto sol_top_left = a_star(source, target, heuristic, BO_LEX_ORDER::LEX1);
    // // cost_t ub1_f = h[source][0] * 1.5;
    // cost_t ub1_f = sol_top_left->f[0];

    // cost_t bucket_width = 1;
    // pqueue_label_bucket<ApexPathPair<N>> open(bucket_width, heuristic(source)[0],
    //                                           ub1_f);

    boost::heap::priority_queue<ApexPathPairPtr<N>, boost::heap::compare<typename ApexPathPair<N>::more_than_full_cost>> open;

    ap = ap_pool.get_label();

    CostVec<N> zero_vec;
    zero_vec.fill(0);

    *ap = ApexPathPair<N>(source, heuristic(source), zero_vec);

    open.push(ap);
    open_map.add(ap);

    while (open.size()) {
        // Pop min from queue and process
        ap = open.top();
        open.pop();
        num_generation += 1;

        if (num_generation % 1000==0 &&
            (clock() - start_time)/CLOCKS_PER_SEC > time_limit
            ){
            break;
        }

        if (!ap->active){
            // ap_pool.save_label(ap);
            continue;
        }

        open_map.remove(ap);

        // Dominance check
        auto tr_f = Tr<N>(ap->f);
        if (gcl_ptr->is_dominated(ap->id, tr_f)){
            ap_pool.save_label(ap);
            continue;
        }

        if (ap_solutions_dr.is_dominated(ap->f, eps)){
            ap_pool.save_label(ap);
            continue;
        }

        num_expansion += 1;

        if (ap->id == target) {
            ap_solutions.push_back(ap);
            ap_solutions_dr.add_gval(ap);
            vector<cost_t> apex(ap->f.begin(), ap->f.end());
            vector<cost_t> cost(ap->path_cost.begin(), ap->path_cost.end());
            this->solutions.emplace_back(apex, cost, clock() - this->start_time);
            continue;
        }

        gcl_ptr->add_gval(ap->id, tr_f);

        CostVec<N> curr_g;
        for (int i = 0; i < N; i ++){
            curr_g[i] = ap->f[i] - heuristic(ap->id)[i];
        }

        // Check to which neighbors we should extend the paths
        const std::vector<Edge> &outgoing_edges = graph[ap->id];
        for (const Edge & e:outgoing_edges){
            size_t next_id = e.target;

            auto next_h = heuristic(next_id);

            CostVec<N> next_f;
            CostVec<N> next_path_cost;
            for (size_t i = 0; i < next_f.size(); i++){
                next_f[i] = curr_g[i] + e.apex[i] + next_h[i];
                next_path_cost[i] = ap->path_cost[i] + e.cost[i];
            }

            auto tr_next_f = Tr<N>(next_f);
            if (gcl_ptr->is_dominated(next_id, tr_next_f)){
                continue;
            }

            if (ap_solutions_dr.is_dominated(next_f, eps)){
                continue;
            }

            next_ap = ap_pool.get_label();
            *next_ap = ApexPathPair<N>(next_id,
                                       next_f,
                                       next_path_cost,
                                       ap);
 
            attempt_merge(next_ap);

            open_map.add(next_ap);
            open.push(next_ap);
            // this->insert(next_ap);
        }
    }
}


template <int N, class GCL>
void ApexSearch<N, GCL>::attempt_merge(ApexPathPairPtr<N> ap){
    if (is == IterOpenStrategy::FILO){
        auto ptr = open_map[ap->id];
        while (ptr!= nullptr){
            if (update_nodes_by_merge_if_bounded<N>(ap, ptr, heuristic(ap->id), eps, ms)){
                open_map.remove(ptr);
                ptr->active = false;
                break; // do we really need to break?
            }
            ptr = ptr->next_in_map_;
        }
    } else if (is == IterOpenStrategy::FIFO){ 
        auto ptr = open_map.get_end(ap->id);
        while (ptr!= nullptr){
            if (update_nodes_by_merge_if_bounded<N>(ap, ptr, heuristic(ap->id), eps, ms)){
                open_map.remove(ptr);
                ptr->active = false;
                break; // do we really need to break?
            }
            ptr = ptr->prev_in_map_;
        }
    } else if (is == IterOpenStrategy::ITER_MAX_SLACK){
        ApexPathPair<N> * to_merge = nullptr;
        double slack = -1;

        auto ptr = open_map[ap->id];
        while (ptr!= nullptr){
            auto new_slack = delta_slack<N>(ap, ptr, heuristic(ap->id), eps);
            if (new_slack >= 0 && new_slack > slack){
                to_merge = ptr;
                slack = new_slack;
            }
            ptr = ptr->next_in_map_;
        }
        if (to_merge != nullptr){
            update_nodes_by_merge_if_bounded<N>(ap, to_merge, heuristic(ap->id), eps, ms);
            open_map.remove(to_merge);
            to_merge->active = false;
        }

    }


}

 template <int N, class GCL>
std::string ApexSearch<N, GCL>::get_solver_name() {
    std::string alg_variant;
    if (ms == MergeStrategy::SMALLER_G2) {
        alg_variant = "-s2";
    } else if (ms == MergeStrategy::SMALLER_G2_FIRST) {
        alg_variant = "-s2f";
    } else if (ms == MergeStrategy::RANDOM) {
        alg_variant = "-r";
    } else if (ms == MergeStrategy::MORE_SLACK) {
        alg_variant = "-ms";
    } else if (ms == MergeStrategy::REVERSE_LEX) {
        alg_variant = "-rl";
    }
    return base_alg_name() + alg_variant;
}

template <int N, class GCL>
void ApexSearch<N, GCL>::init_search() {
    AbstractSolver::init_search();
}


template class SolutionSetTr<2>;
template class SolutionSetTr<3>;
template class SolutionSetTr<4>;
template class ApexSearch<2, G2min>;
template class ApexSearch<3, GCL_array<2>>;
template class ApexSearch<2, GCL_array_hash<1>>;
template class ApexSearch<3, GCL_array_hash<2>>;
template class ApexSearch<4, GCL_array_hash<3>>;
template class ApexSearch<5, GCL_array_hash<4>>;
template class ApexSearch<6, GCL_array_hash<5>>;

// template class ApexSearch<4, GCL_array<3>>;

std::shared_ptr<AbstractSolver> get_Astarpex_solver(AdjacencyMatrix &graph, AdjacencyMatrix &inv_graph,
                                                    size_t source, size_t target, EPS eps
                                                    ){
    if (graph.get_num_of_objectives() == 2){
        return std::make_shared<ApexSearch<2, G2min>>(graph, inv_graph, source, target, eps);
    } else if (graph.get_num_of_objectives() == 3){
        return std::make_shared<ApexSearch<3, GCL<2>>>(graph, inv_graph, source, target, eps); // 
    } else if (graph.get_num_of_objectives() == 4){
        return std::make_shared<ApexSearch<4, GCL_array<3>>>(graph, inv_graph, source, target, eps);
    } else if (graph.get_num_of_objectives() == 5){
        return std::make_shared<ApexSearch<5, GCL_array<4>>>(graph, inv_graph, source, target, eps);
    } else if (graph.get_num_of_objectives() == 6){
        return std::make_shared<ApexSearch<6, GCL_array<5>>>(graph, inv_graph, source, target, eps);
    } else {
        std::cerr << "no supported solver" << std::endl;
        exit(-1);
    }
}
