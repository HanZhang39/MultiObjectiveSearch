
 #include "LTMOA2.h"
#include "AbstractSolver.h"
#include "Utils/Definitions.h"
#include <boost/pool/simple_segregated_storage.hpp>
#include <boost/pool/object_pool.hpp>
#include <cstddef>
#include <ctime>

const size_t max_id = 1000000000;

template <int N, class GCL>
void LTMOA2<N, GCL>::solve(unsigned int time_limit) {
    auto start_time = std::clock();
    // boost::object_pool<Node<N>> storage{30000000,0};

    using NodeN = Node<N>;

    Node<N>* node;
    Node<N>* next;

    typename NodeN::more_than_full_cost more_than;
    std::vector<Node<N>*> open;
    open.reserve(40000000);
    std::make_heap(open.begin(), open.end(), more_than);

    std::array<cost_t, N> a;
    a.fill(0);

    node = new Node<N>(source, a, heuristic(source));
    node->parent = source;
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
        gcl.add_gval(node->id, node->parent, tr_f);

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
            if (next_id == node->parent){continue;}
            std::array<cost_t, N> next_f;
            auto next_h = heuristic(next_id);
            for (size_t i = 0; i < next_f.size(); i++){
                next_f[i] = curr_g[i] + p_edge->cost[i] + next_h[i];
            }

            bool should_check_sol = true;
            if (use_R2 && next_f == node->f){
                should_check_sol = false;
            }

            if (is_dominated(next_id, node->id, should_check_sol, next_f)){
                continue;
            }
            // if (is_dominated_gen(target, max_id ,next_f)){
            //     continue;
            // }
            next = new Node<N>(next_id, next_f);
            next->parent = node->id;
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
bool LTMOA2<N, GCL>::is_dominated(size_t id, size_t parent, bool check_sol, std::array<cost_t, N - 1> tr_f){
    if (!check_sol){
        return gcl.is_dominated(id, parent, tr_f);
    }
    bool res = gcl.is_dominated(id, parent, tr_f) || gcl.is_dominated(target, max_id, tr_f);

    // dominance_check_time += std::clock() - start;
    return res;
}
template <int N, class GCL>
bool LTMOA2<N, GCL>::is_dominated(size_t id, size_t parent, bool check_sol, std::array<cost_t, N> f_val){
    std::array<cost_t, N - 1> tr_f;
    std::copy(std::begin(f_val) + 1, std::end(f_val), std::begin(tr_f));
    return is_dominated(id, parent, check_sol, tr_f);
}

template <int N, class GCL>
bool LTMOA2<N, GCL>::is_dominated(Node<N>* node){
    std::array<cost_t, N - 1> tr_f;
    std::copy(std::begin(node->f) + 1, std::end(node->f), std::begin(tr_f));
    return is_dominated(node->id, node->parent, node->should_check_sol, tr_f);
}


template class LTMOA2<2, GCL_table<1>>;
template class LTMOA2<3, GCL_table<2>>;
template class LTMOA2<4, GCL_table<3>>;
template class LTMOA2<5, GCL_table<4>>;


template class LTMOA2<2, GCL_table_bucket<1>>;
template class LTMOA2<3, GCL_table_bucket<2>>;
template class LTMOA2<4, GCL_table_bucket<3>>;
template class LTMOA2<5, GCL_table_bucket<4>>;

template <int N>
std::shared_ptr<AbstractSolver> __get_LTMOA2_solver__(AdjacencyMatrix &graph, AdjacencyMatrix &inv_graph,
                                                        size_t source, size_t target, 
                                                        bool use_bucket
                                                        ){
    EPS eps(graph.get_num_of_objectives(), 0);
    if (use_bucket){
        return std::make_shared<LTMOA2<N, GCL_table_bucket<N-1>>>(graph, inv_graph, source, target, eps);
    } else {
        
        return std::make_shared<LTMOA2<N, GCL_table<N-1>>>(graph, inv_graph, source, target, eps);
    }
}


std::shared_ptr<AbstractSolver> get_LTMOA2_solver(AdjacencyMatrix &graph, AdjacencyMatrix &inv_graph,
                                                    size_t source, size_t target, 
                                                    bool use_bucket
                                                    ){

    if (graph.get_num_of_objectives() == 2){
        return __get_LTMOA2_solver__<2>(graph,
                                          inv_graph,
                                          source, target,
                                          use_bucket);
    } else if (graph.get_num_of_objectives() == 3){
        return __get_LTMOA2_solver__<3>(graph,
                                          inv_graph,
                                          source, target,
                                          use_bucket);
    } else if (graph.get_num_of_objectives() == 4){
        return __get_LTMOA2_solver__<4>(graph,
                                          inv_graph,
                                          source, target,
                                          use_bucket);
    } else if (graph.get_num_of_objectives() == 5){
        return __get_LTMOA2_solver__<5>(graph,
                                          inv_graph,
                                          source, target,
                                          use_bucket);
    } else if (graph.get_num_of_objectives() == 6){
        return __get_LTMOA2_solver__<6>(graph,
                                          inv_graph,
                                          source, target,
                                          use_bucket);
    } else {
        std::cerr << "no supported solver" << std::endl;
        exit(-1);
    }
}

bool use_R2 = false;
