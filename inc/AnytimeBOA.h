#pragma once

#include "BOAStar.h"
#include "Utils/Definitions.h"
#include "Utils/pool.h"


class Interval{
public:
    double eps = 0;
    NodePtr2 top_left;
    NodePtr2 bottom_right;
    std::shared_ptr<std::list<NodePtr2>> to_expand;

    Interval(){};
    Interval(const NodePtr2 top_left, const NodePtr2 bottom_right, std::shared_ptr<std::list<NodePtr2>> to_expand);
};

using IntervalList   = std::vector<Interval>;

class BOAStarContinuing: public BOAStar<PQ> {
private:

public:

    virtual std::string get_solver_name() {return "BOA* Continuing"; }

    // std::vector<std::pair<std::clock_t, NodePtr2>> solution_log;

    BOAStarContinuing(const AdjacencyMatrix &adj_matrix, double eps);
    BOAStarContinuing(AdjacencyMatrix &graph, AdjacencyMatrix & inv_graph, size_t source, size_t target): BOAStar<PQ>(graph, inv_graph, source, target) {}

    void solve(Interval source, IntervalList & solutions, unsigned int time_limit=UINT_MAX);

    friend class AnytimeBOA;
};

class AnytimeBOA : public AbstractSolver {
protected:
    double d;
    const size_t pool_size = 1024;
    pool<Node<2>> node_pool;
    
public:

    ShortestPathHeuristic<2> heuristic;
    virtual std::string get_solver_name()  override {return "Anytime BOA"; }

    AnytimeBOA(AdjacencyMatrix &graph, AdjacencyMatrix & inv_graph,
               size_t source, size_t target, double d=4):
        AbstractSolver(graph, source, target, {0, 0}),
        heuristic(target, inv_graph),
        node_pool(pool_size),
        d(d)
    {}

    void log_solution(NodePtr2 node);

    void solve(Interval source, IntervalList & solutions, double eps, unsigned int time_limit=UINT_MAX);

    void solve(unsigned int time_limit=UINT_MAX)  override;
};

