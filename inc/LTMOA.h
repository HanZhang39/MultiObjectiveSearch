#pragma once

#include <cstddef>
#include <memory>
#include <vector>
#include "Utils/Definitions.h"
#include "ShortestPathHeuristic.h"
#include "AbstractSolver.h"
#include "GCL.h"

template <int N, class GCL>
class LTMOA: public AbstractSolver {
protected:

    std::list<Node<N>*> nodes;
    using GCL_ptr = std::unique_ptr<GCL>;

    GCL_ptr gcl_ptr;
    // virtual bool is_dominated_gen(Node<N>* node){return is_dominated(node);};
    virtual bool is_dominated_gen(size_t id, std::array<cost_t, N> f_val, bool check_global){return is_dominated(id, f_val, check_global);};
    virtual bool is_dominated(Node<N>* node);
    virtual bool is_dominated(size_t id, std::array<cost_t, N> f_val, bool check_global);
    virtual bool is_dominated(size_t id, std::array<cost_t, N - 1> tr_f, bool check_global);

    std::clock_t queue_time = 0;
    std::clock_t dominance_check_time = 0;

public:

    ShortestPathHeuristic<N> heuristic;
    LTMOA(AdjacencyMatrix &adj_matrix, AdjacencyMatrix &inv_graph, size_t source, size_t target, EPS eps):
        AbstractSolver(adj_matrix, source, target, eps),
        gcl_ptr(std::make_unique<GCL>(adj_matrix.size())),
        heuristic(ShortestPathHeuristic<N>(target, inv_graph))
    {}

    virtual std::string get_solver_name() override {return "LTMOA(" + std::to_string(N)+")-" + gcl_ptr->get_name(); }

    void solve(unsigned int time_limit=UINT_MAX) override;

    virtual ~LTMOA(){
        for (auto ptr:nodes){
            delete ptr;
        }
    }

};



template <int N, class GCL>
class LazyLTMOA: public LTMOA<N, GCL> {

    virtual bool is_dominated_gen(size_t id, std::array<cost_t, N> f_val, bool check_global) override{ return false;};
public:

    LazyLTMOA(AdjacencyMatrix &adj_matrix, AdjacencyMatrix & inv_graph, size_t source, size_t target, EPS eps):
        LTMOA<N, GCL> (adj_matrix, inv_graph, source, target, eps)
    {}

    virtual std::string get_solver_name() override {return "Lazy" + LTMOA<N, GCL>::get_solver_name(); }

};

std::shared_ptr<AbstractSolver>
get_LTMOA_solver(AdjacencyMatrix &graph, AdjacencyMatrix &inv_graph,
                 size_t source, size_t target, int type = 0, std::vector<double> eps={});

extern bool use_R2;
