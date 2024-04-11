#include "AnytimeApexSearch.h"
#include "ApexSearch.h"
#include "Utils/Definitions.h"
#include "Utils/common.h"
#include <unordered_map>
#include <boost/unordered_set.hpp>
#include <unordered_set>
#include <vector>

// template class PrunedNodeTable<3>;


template <int N>
ApexSearchContinue<N>::ApexSearchContinue(AdjacencyMatrix &graph,
                                          AdjacencyMatrix & inv_graph,
                                          size_t source,
                                          size_t target,
                                          EPS eps,
                                          bool full_gcl):
        ApexSearch<N, GCL_array_hash<N-1>>(graph, inv_graph, source, target, eps),
        gcl_apex(this->graph.size()),
        use_gcl_full(full_gcl),
        full_gcl_ptr(std::make_unique<GCL_array<N>>(graph.size())),
        update_gcl_full(full_gcl)
    {
        pruned_list.reserve(50000);
        restart_from_scratch();

    }


template <int N> void ApexSearchContinue<N>::restart_from_scratch() {
    CostVec<N> zero_vec;
    zero_vec.fill(0);
    pruned_list.clear();
    pruned_list.emplace_back(this->source, zero_vec, 0);
    full_gcl_ptr->reset();

}


template <int N>
void ApexSearchContinue<N>::init_open(boost::heap::priority_queue<ApexPathPairPtr<N>, boost::heap::compare<typename ApexPathPair<N>::more_than_full_cost>> & open){
    ApexPathPairPtr<N> next_ap = nullptr;
    CostVec<N> next_f;

    if (this->verbal >= 1){
        cout << "pruned_list size: " << pruned_list.size() << endl;
    }

    std::vector<CachedNode<N>> pruned_list_;
    for (auto & cached: pruned_list){
        auto next_id = cached.id;
        next_ap = this->ap_pool.get_label();
        for(int i=0; i < N; i++){
            next_f[i] = this->heuristic(next_id)[i] + cached.g[i];
        }
        *next_ap = ApexPathPair<N>(next_id, next_f, cached.g);

        auto ptr = this->open_map[next_id];

        while (ptr!= nullptr){
            if (update_nodes_by_merge_if_bounded<N>(next_ap, ptr, this->heuristic(next_id), this->eps, this->ms)){
                this->open_map.remove(ptr);
                if (!is_dominating<N>(next_ap->path_cost, ptr->path_cost)){
                    pruned_list_.emplace_back(next_id, ptr->path_cost, -1);
                }
                ptr->active = false;
                break; // do we really need to break?
            }
            ptr = ptr->next_in_map_;
        }

        this->open_map.add(next_ap);
        open.push(next_ap);
    }
    pruned_list.clear();
    if (this->verbal >= 1){
        cout << "pruned_list_ size: " << pruned_list_.size() << endl;
    }
    pruned_list = pruned_list_;

    for (auto sol: this->ap_solutions){
        for (int i = 0; i < N; i++){
            sol->f[i] = ((double) sol->path_cost[i])/(1 + this->eps[i]);
        }
        sol->f = sol->path_cost;
        // sol->id = this->IDX_PREV_SOLUTION;

        this->open_map.add(sol);
        open.push(sol);
    }

    this->ap_solutions.clear();
    this->ap_solutions_dr.solutions.clear();
    this->solutions.clear();

}

template <int N>
void ApexSearchContinue<N>::solve(unsigned int time_limit){
    
    this->init_search();

    ApexPathPairPtr<N> ap = nullptr;
    ApexPathPairPtr<N> next_ap = nullptr;

    CostVec<N> path_f;
    CostVec<N> curr_g;
    CostVec<N> next_f;
    CostVec<N> next_path_cost;

    boost::heap::priority_queue<ApexPathPairPtr<N>, boost::heap::compare<typename ApexPathPair<N>::more_than_full_cost>> open;

    init_open(open);

    while (open.size()) {
        // Pop min from queue and process
        ap = open.top();
        open.pop();
        this->num_generation += 1;

        if (this->num_generation % 1000==0 &&
            (clock() - this->start_time)/CLOCKS_PER_SEC > time_limit
            ){
            break;
        }

        if (!ap->active){
            this->ap_pool.save_label(ap);
            continue;
        }

        this->open_map.remove(ap);

        // Dominance check
        if (gcl_apex.is_dominated(ap->id, ap->f)){
            if (!is_dominating<N>(gcl_apex.dominating->path_cost, ap->path_cost)){
                insert_to_pruned(ap->id, ap->path_cost);
            }
            this->ap_pool.save_label(ap);
            continue;
        }
        if (this->ap_solutions_dr.is_dominated(ap->f, this->eps)){
            for (int i = 0; i < N; i ++){
                path_f[i] = ap->path_cost[i] + this->heuristic(ap->id)[i];
            }
            if (!is_dominating<N>(this->ap_solutions_dr.dominating->path_cost, path_f)){
                insert_to_pruned(ap->id, ap->path_cost);
            }
            this->ap_pool.save_label(ap);
            continue;
        }

        for (int i = 0; i < N; i ++){
            curr_g[i] = ap->f[i] - this->heuristic(ap->id)[i];
        }


        if (use_gcl_full && ap->id != this->target && full_gcl_ptr->is_dominated(ap->id, ap->path_cost)){
            continue;
        }

        this->num_expansion += 1;

        if (ap->id == this->target) {
            this->ap_solutions.push_back(ap);
            this->ap_solutions_dr.add_gval(ap);

            vector<cost_t> apex(ap->f.begin(), ap->f.end());
            vector<cost_t> cost(ap->path_cost.begin(), ap->path_cost.end());
            this->solutions.emplace_back(apex, cost, clock() - this->start_time);


            continue;
        }

        gcl_apex.add_gval(ap);
        if (update_gcl_full && ap->id != this->target){
            full_gcl_ptr->add_gval(ap->id, ap->path_cost);
        }


        if (update_gcl_full_direct && ap->id != this->target){
            full_gcl_ptr->add_gval_direct(ap->id, ap->path_cost);
        }


        // Check to which neighbors we should extend the paths
        const std::vector<Edge> &outgoing_edges = this->graph[ap->id];
        for (const Edge & e:outgoing_edges){
            size_t next_id = e.target;

            auto next_h = this->heuristic(next_id);

            for (size_t i = 0; i < next_f.size(); i++){
                next_f[i] = curr_g[i] + e.apex[i] + next_h[i];
                next_path_cost[i] = ap->path_cost[i] + e.cost[i];
            }

            if (gcl_apex.is_dominated(next_id, next_f)){
                if (!is_dominating<N>(gcl_apex.dominating->path_cost, next_path_cost)){
                    insert_to_pruned(next_id, next_path_cost);
                }
                continue;
            }

            if (this->ap_solutions_dr.is_dominated(next_f, this->eps)){
                for (int i = 0; i < N; i ++){
                    path_f[i] = next_path_cost[i] + next_h[i];
                }
                if (!is_dominating<N>(this->ap_solutions_dr.dominating->path_cost, path_f)){
                    insert_to_pruned(next_id, next_path_cost);
                }
                continue;
            }

            next_ap = this->ap_pool.get_label();
            *next_ap = ApexPathPair<N>(next_id,
                                       next_f,
                                       next_path_cost,
                                       ap);
 
            auto ptr = this->open_map[next_id];

            while (ptr!= nullptr){
                if (update_nodes_by_merge_if_bounded<N>(next_ap, ptr, this->heuristic(next_id), this->eps, this->ms)){
                    this->open_map.remove(ptr);

                    if (!is_dominating<N>(next_ap->path_cost, ptr->path_cost)){
                        insert_to_pruned(next_id, ptr->path_cost);
                    }

                    ptr->active = false;
                    break; // do we really need to break?
                }
                ptr = ptr->next_in_map_;
            }


            this->open_map.add(next_ap);
            open.push(next_ap);
            // this->insert(next_ap);
        }
    }

    gcl_apex.clear();
}

template <int N>
void ApexSearchContinue<N>::insert_to_pruned(size_t state, const CostVec<N>& gval){
    CostVec<N> gval_copy = gval;
    pruned_list.emplace_back(state, gval_copy, -1);
}


template <int N>
void AnytimeApex<N>::solve(unsigned int time_limit){

    start_time = std::clock();
    search.set_start_time(start_time);

    double eps = 0.2;

    bool restart = restart_from_scratch;

    while(true){
    // for (auto eps: eps_seq){
        if (verbal > 0){
            std::cout << std::endl << std::endl;
            std::cout << "eps:" << eps << std::endl;
        }

        EPS eps_vec(N, eps);
        search.set_eps(eps_vec);

        double next_eps = eps/ decrease_factor;

        // v1 restart rule
        // if (next_eps < hybrid_thr){
        //     search.update_gcl_full = true;
        // }

        search.solve(time_limit);

        if (verbal > 0){
            std::cout << "elapse time: " << ((double)std::clock() - start_time)/CLOCKS_PER_SEC << std::endl;
            std::cout << "exp: " << search.get_num_expansion() << ",   pruned: " << search.pruned_list.size( );

        }

        num_expansion += search.get_num_expansion();
        num_generation += search.get_num_generation();


        solution_log.insert(solution_log.end(), search.solutions.begin(), search.solutions.end());
        if ((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){
            solutions.insert(solutions.end(), search.solutions.begin(), search.solutions.end());
            break;
        } else {
            solutions = search.solutions;
        }

        if (!search.is_approximate()){
            break;
        }

        auto num_exp_ = search.get_num_expansion();
        auto num_prune_ = search.pruned_list.size();

        if (is_hybrid && num_exp_ > hybrid_param * num_prune_ && restart){
            restart = false;
            search.update_gcl_full = true;
            search.update_gcl_full_direct = false;
            search.use_gcl_full = true;
            // cout << "stop restarting" << endl;
        }

        if (restart){
            // cout << "restarting" << endl;
            search.restart_from_scratch();
        }


        eps = next_eps;
    }
}


template <int N>
SolutionSet AnytimeApex<N>::get_solution_log(){
    SolutionSet filtered;
    boost::unordered_set<CostVec<N>> cost_found;
    CostVec<N> cost;

    for (auto& sol: solution_log){
        for (int i = 0; i < N; i++){
            cost[i] = sol.cost[i];
        }

        if (cost_found.find(cost) == cost_found.end()){
            filtered.push_back(sol);
            cost_found.insert(cost);
        }
    }

    return filtered;
}


template class ApexSearchContinue<3>;
template class ApexSearchContinue<4>;
template class ApexSearchContinue<5>;

template class AnytimeApex<3>;
template class AnytimeApex<4>;
template class AnytimeApex<5>;



template <int N>
std::shared_ptr<AbstractSolver> _get_anytime_Apex_solver_(AdjacencyMatrix &graph, AdjacencyMatrix &inv_graph,
                                                        size_t source, size_t target, EPS eps, int type                                                   ){

    std::shared_ptr<AbstractSolver> res = nullptr;
    if (type == 1){
        auto ptr = std::make_shared<ApexSearchContinue<N>>(graph, inv_graph, source, target, eps);
        ptr->print_stat_flag = true;
        res = ptr;
    } else {
        bool use_full_gcl = type == 4;
        auto ptr = std::make_shared<AnytimeApex<N>>(graph, inv_graph, source, target, eps, use_full_gcl);
        ptr->restart_from_scratch = type == 3;
        if (type == 5){
            ptr->hybrid_param= default_hybrid_param;
            ptr->is_hybrid = true;
            ptr->restart_from_scratch = true;
        }
        res = ptr;
    }

    return res;
}

std::shared_ptr<AbstractSolver> get_anytime_Apex_solver(AdjacencyMatrix &graph, AdjacencyMatrix &inv_graph,
                                                        size_t source, size_t target, EPS eps, int type                                                   ){
    if (graph.get_num_of_objectives() == 2){
        return _get_anytime_Apex_solver_<2>(graph, inv_graph, source, target, eps, type);
    } else if (graph.get_num_of_objectives() == 3){
        return _get_anytime_Apex_solver_<3>(graph, inv_graph, source, target, eps, type);
    } else if (graph.get_num_of_objectives() == 4){
        return _get_anytime_Apex_solver_<4>(graph, inv_graph, source, target, eps, type);
    } else if (graph.get_num_of_objectives() == 5){
        return _get_anytime_Apex_solver_<5>(graph, inv_graph, source, target, eps, type);
    } else if (graph.get_num_of_objectives() == 6){
        return _get_anytime_Apex_solver_<6>(graph, inv_graph, source, target, eps, type);
    } else {
        std::cerr << "no supported solver" << std::endl;
        exit(-1);
    }
}

double default_hybrid_param = 5;
double decrease_factor = 4;
