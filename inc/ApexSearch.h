#pragma once
#include "ShortestPathHeuristic.h"
#include "Utils/Definitions.h"
#include"GCL.h"
#include "AbstractSolver.h"
#include <unordered_map>
#include <vector>

#include "Utils/pool.h"

enum MergeStrategy {
    SMALLER_G2,
    RANDOM,
    MORE_SLACK,
    SMALLER_G2_FIRST,
    REVERSE_LEX,
    LEX
};


enum IterOpenStrategy{
    FILO,
    FIFO,
    ITER_MAX_SLACK
};

const size_t POOL_SIZE = 1024;

template<int N>
bool is_bounded(CostVec<N> & apex, CostVec<N> & pathcost, CostVec<N> & h, const EPS eps);

template <int N>
double compute_slack(CostVec<N> &apex, CostVec<N> &path_cost, CostVec<N> & h,
                     const EPS eps);
template<size_t N>
class SolutionSetTr{
protected:
public:
    std::vector<ApexPathPair<N> *> solutions;

    ApexPathPair<N>* dominating;

    SolutionSetTr() {
    };

    inline bool is_dominated(std::array<cost_t, N> & fval, const vector<double>& eps) {
        for (auto sol_ptr: solutions){
            if(is_dominating_dr<N>(sol_ptr->path_cost, fval, eps)){
                // update apex
                for (int i = 0; i < N; i++){
                    sol_ptr->f[i] = min(sol_ptr->f[i], fval[i]);
                }
                dominating = sol_ptr;
                return true;
            }
        }

        return false;
    }

    inline bool is_dominated_apex(std::array<cost_t, N> & fval) {
        for (auto sol_ptr: solutions){
            if(is_dominating_dr<N>(sol_ptr->f, fval)){
                // update apex

                return true;
            }
        }

        return false;
    }

    inline bool is_dominated_exact(std::array<cost_t, N> & fval) {
        for (auto sol_ptr: solutions){
            if(is_dominating_dr<N>(sol_ptr->path_cost, fval)){
                return true;
            }
        }
        return false;
    }

    inline void add_gval(ApexPathPair<N> * solution){
        solutions.push_back(solution);

        for (int i = solutions.size() - 2 ; i >=0 ;i--){
            if (is_dominating_dr<N>(solution->f, solutions[i]->f)){
                solutions[i] = solutions.back();
                solutions.pop_back();
            }
        }
    }
};


template <int N>
struct OpenMap {
    std::vector<ApexPathPair<N>*> open_map;
    std::vector<ApexPathPair<N>*> open_map_end;

    OpenMap<N>(int size):
        open_map(size, nullptr),
        open_map_end(size, nullptr)
    {}

    ApexPathPair<N>* & operator[](int id){
        return open_map[id];
    }


    ApexPathPair<N>* & get_end(int id){
        return open_map_end[id];
    }

    inline void add(ApexPathPairPtr<N> ap){
        if (open_map[ap->id] != nullptr){
            open_map[ap->id] -> prev_in_map_ = ap;
        } else {
            open_map_end[ap->id] = ap;
        }
        ap->next_in_map_ = open_map[ap->id];
        ap->prev_in_map_ = nullptr;
        open_map[ap->id] = ap;
    }

    inline void remove(ApexPathPairPtr<N> ap) {
        // if (open_map[ap->id] == ap){
        //     if (ap->next_in_map_ != nullptr){
        //         ap->next_in_map_->prev_in_map_ = nullptr;
        //     }
        //     open_map[ap->id] = ap->next_in_map_;
        //     ap->next_in_map_ = nullptr;
        // } else {
        //     ap->prev_in_map_->next_in_map_ = ap->next_in_map_;
        //     if (ap->next_in_map_!= nullptr){
        //       ap->next_in_map_->prev_in_map_ = ap->prev_in_map_;
        //     }
        //     ap->prev_in_map_ = nullptr;
        //     ap->next_in_map_ = nullptr;
        // }

        if (open_map[ap->id] == ap){
            // if (ap->next_in_map_ != nullptr){
            //     ap->next_in_map_->prev_in_map_ = nullptr;
            // }
            open_map[ap->id] = ap->next_in_map_;
            // ap->next_in_map_ = nullptr;
        }

        if (open_map_end[ap->id] == ap){
            open_map_end[ap->id] = ap->prev_in_map_;
        }

        if (ap->prev_in_map_!= nullptr){
            ap->prev_in_map_->next_in_map_ = ap->next_in_map_;
        }

        if (ap->next_in_map_!= nullptr){
            ap->next_in_map_->prev_in_map_ = ap->prev_in_map_;
        }
        ap->prev_in_map_ = nullptr;
        ap->next_in_map_ = nullptr;

    }

};

template< int N>
bool update_nodes_by_merge_if_bounded(const ApexPathPairPtr<N> & ap, const ApexPathPairPtr<N> &other, CostVec<N> h, const std::vector<double> eps, MergeStrategy s);


template <int N, class GCL>
class ApexSearch: public AbstractSolver {
protected:

    using GCL_ptr = std::unique_ptr<GCL>;
    GCL_ptr gcl_ptr;
    SolutionSetTr<N> ap_solutions_dr;
    std::vector<ApexPathPair<N>*> ap_solutions;

    OpenMap<N> open_map;

    pool<ApexPathPair<N>> ap_pool = pool<ApexPathPair<N>>(POOL_SIZE);

protected:


    void attempt_merge(ApexPathPairPtr<N> ap);

    ShortestPathHeuristic<N> heuristic;

    size_t num_of_objectives;
    MergeStrategy ms = MergeStrategy::MORE_SLACK;
    IterOpenStrategy is = IterOpenStrategy::FILO;

    // virtual void insert(ApexPathPairPtr<N> &ap);
    // virtual bool is_dominated(ApexPathPairPtr<N> ap);
    // virtual void merge_to_solutions(const ApexPathPairPtr<N> &pp);
    void init_search() override;

    virtual std::string base_alg_name() { return "A*pex"; }

public:

    virtual std::string get_solver_name() override;


    void set_merge_strategy(MergeStrategy new_ms){ms = new_ms;}
    ApexSearch(AdjacencyMatrix &graph, AdjacencyMatrix & inv_graph,
               size_t source, size_t target,
               EPS eps);
    virtual void solve(unsigned int time_limit=UINT_MAX) override;
    // virtual void operator()(size_t source, size_t target, Heuristic &heuristic, std::vector<ApexPathPairPtr<N>> &solutions, unsigned int time_limit=UINT_MAX);
};

// template class ApexSearch<2>;

std::shared_ptr<AbstractSolver> get_Astarpex_solver(AdjacencyMatrix &graph, AdjacencyMatrix &inv_graph,
                                                    size_t source, size_t target, EPS eps                                                    );



template <int N>
struct OpenMapHash {
    std::unordered_map<size_t, ApexPathPair<N>*> open_map;


    OpenMapHash<N>(int size=0){}

    ApexPathPair<N>*  operator[](int id){
        if (open_map.find(id) == open_map.end()){
            return nullptr;
        }
        return open_map[id];
    }

    inline void add(ApexPathPairPtr<N> ap){
        if (open_map.find(ap->id) == open_map.end()){
            open_map[ap->id] = nullptr;
        }

        if (open_map[ap->id] != nullptr){
            open_map[ap->id] -> prev_in_map_ = ap;

            }
        ap->next_in_map_ = open_map[ap->id];
        open_map[ap->id] = ap;
    }

    inline void remove(ApexPathPairPtr<N> ap) {
        if (open_map[ap->id] == ap){
            if (ap->next_in_map_ != nullptr){
                ap->next_in_map_->prev_in_map_ = nullptr;
            }
            open_map[ap->id] = ap->next_in_map_;
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

};
