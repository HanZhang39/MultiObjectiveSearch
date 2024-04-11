 #include "LTMOA.h"
#include "GCL.h"
#include "GCL_bucket.h"
// #include "GCL_ndtree.h"
#include "Utils/Definitions.h"
#include <boost/pool/simple_segregated_storage.hpp>
#include <boost/pool/object_pool.hpp>
#include <cstddef>
#include <ctime>
#include <vector>

template <int N, class GCL>
void LTMOA<N, GCL>::solve(unsigned int time_limit) {
    auto start_time = std::clock();
    // boost::object_pool<Node<N>> storage{30000000,0};

    using NodeN = Node<N>;

    Node<N>* node;
    Node<N>* next;

    typename NodeN::more_than_full_cost more_than;
    std::vector<Node<N>*> open;
    std::make_heap(open.begin(), open.end(), more_than);

    std::array<cost_t, N> a;
    a.fill(0);

    node = new Node<N>(source, a, heuristic(source));
    // node = storage.construct(source, a, heuristic(source));

    nodes.push_back(node);
    open.push_back(node);
    std::push_heap(open.begin(), open.end(), more_than);

    // std::clock_t start;

    while (open.empty() == false) {
        if ((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){
            return;
        }
        // Pop min from queue and process

        std::pop_heap(open.begin(), open.end(), more_than);
        node = open.back();
        open.pop_back();

        num_generation +=1;

        if (is_dominated(node)){
            continue;
        }

        std::array<cost_t, N - 1> tr_f;
        std::copy(std::begin(node->f) + 1, std::end(node->f), std::begin(tr_f));
        gcl_ptr->add_gval(node->id, tr_f);

        num_expansion += 1;

        if (node->id == target) {
            vector<cost_t> apex(node->f.begin(), node->f.end());
            solutions.emplace_back(apex, apex, clock()- start_time);
            continue;
        }

        auto curr_h = heuristic(node->id);
        std::array<cost_t, N> curr_g;
        for (int i = 0 ; i < N; i++){
            curr_g[i] = node->f[i] - curr_h[i] ;
        }

        const std::vector<Edge> &outgoing_edges = graph[node->id];
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            size_t next_id = p_edge->target;
            std::array<cost_t, N> next_f;
            auto next_h = heuristic(next_id);
            for (size_t i = 0; i < next_f.size(); i++){
                next_f[i] = curr_g[i] + p_edge->cost[i] + next_h[i];
            }

            bool should_check_sol = true;
            if (use_R2 && next_f == node->f){
                should_check_sol = false;
            }

            if (is_dominated_gen(next_id, next_f, should_check_sol)){
                continue;
            }

            next = new Node<N>(next_id, next_f);
            next->should_check_sol = should_check_sol;

            nodes.push_back(next);
            open.push_back(next);
            std::push_heap(open.begin(), open.end(), more_than);
        }
    }

    std::cout << "queue time: " << queue_time/CLOCKS_PER_SEC << std::endl;
    std::cout << "dominance check time: " << dominance_check_time/CLOCKS_PER_SEC << std::endl;

}


template <int N, class GCL>
bool LTMOA<N, GCL>::is_dominated(size_t id, std::array<cost_t, N - 1> tr_f, bool check_global){
    if (!check_global){
        return gcl_ptr->is_dominated(id, tr_f);
    }
    std::array<cost_t, N - 1> inflated_tr_f;
    for (int i = 0; i < N-1; i++){
        inflated_tr_f[i] = tr_f[i] * (1 + eps[i + 1]);
    }
    bool res = gcl_ptr->is_dominated(id, tr_f) || gcl_ptr->is_dominated(target, inflated_tr_f);

    // dominance_check_time += std::clock() - start;
    return res;
}
template <int N, class GCL>
bool LTMOA<N, GCL>::is_dominated(size_t id, std::array<cost_t, N> f_val, bool check_global){
    std::array<cost_t, N - 1> tr_f;
    std::copy(std::begin(f_val) + 1, std::end(f_val), std::begin(tr_f));
    return is_dominated(id, tr_f, check_global);
}

template <int N, class GCL>
bool LTMOA<N, GCL>::is_dominated(Node<N>* node){
    std::array<cost_t, N - 1> tr_f;
    std::copy(std::begin(node->f) + 1, std::end(node->f), std::begin(tr_f));
    return is_dominated(node->id, tr_f, node->should_check_sol);
}


template class LTMOA<2, GCL<1>>;
template class LTMOA<3, GCL<2>>;
template class LTMOA<3, GCL_logtime>;
template class LTMOA<4, GCL<3>>;
template class LTMOA<5, GCL<4>>;

template <int N>
std::shared_ptr<AbstractSolver> __get_LTMOA_solver__(AdjacencyMatrix &graph, AdjacencyMatrix &inv_graph,
                                                     size_t source, size_t target, 
                                                     int type, std::vector<double> eps
                                                   ){
    if (eps.size() == 0){
        eps.resize(N, 0);
    }
    if (type == -1){
        return std::make_shared<LazyLTMOA<N, GCL<N-1>>>(graph, inv_graph, source, target, eps);
    } else if (type == 0){
        return std::make_shared<LTMOA<N, GCL_array<N-1>>>(graph, inv_graph, source, target, eps);
    } else if (type == 1) {
        return std::make_shared<LTMOA<N, GCL_array_bucket<N-1>>>(graph, inv_graph, source, target, eps);
    // } else if (type == 2) {
    //     return std::make_shared<LTMOA<N, GCL_ndtree<N-1>>>(graph, inv_graph, source, target, eps);
    }
    return nullptr;
}

std::shared_ptr<AbstractSolver> get_LTMOA_solver(AdjacencyMatrix &graph, AdjacencyMatrix &inv_graph,
                                                   size_t source, size_t target, 
                                                 int type, std::vector<double> eps
                                                   ){
    
    if (graph.get_num_of_objectives() == 2){
        return __get_LTMOA_solver__<2>(graph,
                                         inv_graph,
                                         source, target, 
                                         type, eps);
    } else if (graph.get_num_of_objectives() == 3){
        return __get_LTMOA_solver__<3>(graph,
                                         inv_graph,
                                         source, target, 
                                         type, eps);
    } else if (graph.get_num_of_objectives() == 4){
        return __get_LTMOA_solver__<4>(graph,
                                         inv_graph,
                                         source, target, 
                                         type, eps);
    } else if (graph.get_num_of_objectives() == 5){
        return __get_LTMOA_solver__<5>(graph,
                                         inv_graph,
                                         source, target, 
                                         type, eps);
    } else if (graph.get_num_of_objectives() == 6){
        return __get_LTMOA_solver__<6>(graph,
                                         inv_graph,
                                         source, target, 
                                         type, eps);
    } else {
        std::cerr << "no supported solver" << std::endl;
        exit(-1);
    }
}
