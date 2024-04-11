#pragma once
#include "ApexSearch.h"
#include "GCL.h"
#include <sstream>


class WCApexSearch : public ApexSearch<2, G2min> {
protected:
    int resource_bound;
    int bound_ratio;

    std::array<cost_t, 2> best_solution_so_far = {MAX_COST, MAX_COST};

    BiobjectiveHeuristic bi_heuristic;

    cost_t lb1_f;
    cost_t ub1_f;
    cost_t lb2_f;
    cost_t ub2_f;



    const AdjacencyMatrix inv_graph;
    BiobjectiveHeuristic inv_bi_heuristic;
    int num_expansion_bwd = 0;
    int num_generation_bwd = 0;

    // TODO remove this...
    std::vector<ApexPathPair<2>*> open_map_bwd;


public:


    virtual std::string get_solver_name() override {
        return "WCApex";
        // return std::format("WCApex({})", bound_ratio);
    }

    std::string get_result_str() override{
        std::stringstream ss;
        ss << get_solver_name() <<  "\t"
           << source << "\t" << target << "\t"
           << get_num_generation() << "\t"
           << get_num_expansion() << "\t"
           << bound_ratio << "\t"
           << resource_bound << "\t"
            ;
        if (solutions.size() == 1){
            ss << solutions.front().cost[0] << "\t" 
               << solutions.front().cost[1] << "\t";
        } else {
            ss << -1 << "\t" 
               << -1 << "\t";
        }

        std::string result = ss.str();
        return result;
    }

    WCApexSearch(AdjacencyMatrix &graph, AdjacencyMatrix &inv_graph,
                 size_t source, size_t target,
                 int bound_ratio, double eps);

    virtual void solve(unsigned int time_limit = UINT_MAX) override;
    virtual void solve_backword(unsigned int time_limit = UINT_MAX);
};




void add_to_openmap_bwd(std::vector<ApexPathPair<2>*> &open_map_bwd, ApexPathPairPtr<2> ap);
void remove_from_openmap_bwd(std::vector<ApexPathPair<2>*> &open_map_bwd, ApexPathPairPtr<2> ap);
