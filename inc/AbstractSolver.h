#pragma once

#include "Utils/Definitions.h"
#include <cstddef>
#include <chrono>

class AbstractSolver {
protected:

    std::clock_t start_time;

    AdjacencyMatrix   &graph;
    // Pair<double>            eps;
    EPS eps;

    size_t num_expansion = 0;
    size_t num_generation= 0;
    std::vector<std::pair<std::clock_t, std::vector<size_t>>> solution_log;

    size_t source;
    size_t target;

    // if parent_solver is not null, the solution will be log to the parent.
    AbstractSolver* parent_solver=nullptr;

    // void log_solution(std::vector<size_t> sol);
    // void log_all_solutions();


    virtual void init_search(){
        num_expansion = 0;
        num_generation = 0;
    }


public:

    std::chrono::time_point<std::chrono::steady_clock> start_time_;

    int verbal=0;

    SolutionSet solutions;

    virtual SolutionSet get_solution_log(){return solutions;}
    virtual std::string get_result_str();
    virtual std::string get_solver_name() = 0;

    size_t get_num_expansion(){return num_expansion;}
    size_t get_num_generation(){return num_generation;}

    virtual void solve(unsigned int time_limit=UINT_MAX){
        std::cerr << "solver not implemented. " << std::endl;
        exit(1);
    };

    void set_parent(AbstractSolver* parent){parent_solver = parent;}

    AbstractSolver(AdjacencyMatrix &adj_matrix, size_t source, size_t target, EPS eps):
        graph(adj_matrix),
        source(source), target(target),
        eps(eps) {
        solutions.clear();
    }
    virtual ~AbstractSolver(){}
};

