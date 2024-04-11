#pragma once
#include "ApexSearch.h"
#include "GCL.h"
#include "Utils/Definitions.h"
#include <cstddef>
#include <memory>
#include <sys/_types/_clock_t.h>
#include <vector>


template <int N>
struct CachedNode{
    size_t id;
    CostVec<N> g;

    // eps value when it is approximately dominated
    double eps;

    CachedNode(size_t id, CostVec<N> & g, double eps):
        id(id), g(g), eps(eps)
    {}
};


template<size_t N>
class GCL_array_apex{
protected:

public:
    std::vector<std::vector<ApexPathPair<N>*>> gcl;
    std::string get_name(){return "array";}

    GCL_array_apex(size_t graph_size):gcl(graph_size + 1) {
        for (int i = 0; i < graph_size; i++){
            gcl[i].reserve(ARRAY_DEFAULT_SIZE);
        }
    };

    void clear(){
        for (int i = 0; i < gcl.size(); i++){
            gcl[i].clear();
        }
    }

    ApexPathPair<N>* dominating;

    inline bool is_dominated(size_t state, std::array<cost_t, N> & gval) {
        // check_cnt += 1;
        if (state >= gcl.size()){
            return false;
        }

        for (auto & ap: gcl[state]){
            if(is_dominating_dr<N>(ap->f, gval)){
                dominating = ap;
                return true;
            }
        }

        return false;
    }

    inline void add_gval(ApexPathPair<N>* ap){
        size_t state = ap->id;
        if (state >= gcl.size()){
            cerr << "call GCL with wrong index" << endl;
            exit(-1);
            return;
        }

        gcl[state].push_back(ap);


        for (int i = gcl[state].size() - 2 ; i >=0 ;i--){
            if (is_dominating_dr<N>(ap->f, gcl[state][i]->f)){
                gcl[state][i] = gcl[state].back();
                gcl[state].pop_back();
            }
      
        }
    }

};

template class GCL_array_apex<2>;
template class GCL_array_apex<3>;



template <int N>
class ApexSearchContinue: public ApexSearch<N, GCL_array_hash<N-1>> {
protected:

    size_t IDX_PREV_SOLUTION = 1000000000;


  GCL_array_apex<N> gcl_apex;

    void init_open(boost::heap::priority_queue<ApexPathPairPtr<N>, boost::heap::compare<typename ApexPathPair<N>::more_than_full_cost>> & open);

public:

    std::unique_ptr<GCL_array<N>> full_gcl_ptr;
    bool use_gcl_full = false;
    bool update_gcl_full = false;
    bool update_gcl_full_direct = true;

    std::vector<CachedNode<N>> pruned_list;

 void insert_to_pruned(size_t state, const CostVec<N>& gval);


    ApexSearchContinue(AdjacencyMatrix &graph, AdjacencyMatrix & inv_graph,
                       size_t source, size_t target,
                       EPS eps, bool full_gcl=false
                       );

    bool print_stat_flag = false;
    ~ApexSearchContinue(){
        if (print_stat_flag){
            cout << "pruned_list_size: " << pruned_list.size() << endl;
        }
    }

    bool is_approximate(){
        return ! pruned_list.empty();
    }
    void set_eps (EPS eps){this->eps = eps;}
    void set_start_time(clock_t start_time){this->start_time = start_time;}
    void restart_from_scratch();

    virtual void solve(unsigned int time_limit=UINT_MAX) override;

} ;

extern double default_hybrid_param;
extern double decrease_factor;

template <int N>
class AnytimeApex: public AbstractSolver {
protected:
    double d;
    ApexSearchContinue<N> search;
    SolutionSet solution_log;

public:

    bool use_gcl = false;
    bool restart_from_scratch=false;

    // double hybrid_thr = 0; // obsolete
    double hybrid_param = 4;
    bool is_hybrid = false;

    AnytimeApex(AdjacencyMatrix & graph, AdjacencyMatrix & inv_graph,
                size_t source, size_t target,
                EPS eps, bool use_full_gcl
                ):
        AbstractSolver(graph, source, target, eps),
        search(graph, inv_graph, source, target, eps, use_full_gcl)
    {
    }
    ;

    virtual void solve(unsigned int time_limit=UINT_MAX) override;
    virtual std::string get_solver_name() override {
        std::string name ="Anytime_Apex" ;
        
        name += (is_hybrid ? "_Hybrid":"");
        name += (!is_hybrid && restart_from_scratch ? "_R":"");
        name += (use_gcl ? "_G":"");
        return name;
    }

    void set_use_gcl(bool flag){
        use_gcl = flag;
    }

    virtual SolutionSet get_solution_log() override;
};



/*
 *
 type = 1 ApexSearchContinue

 */

std::shared_ptr<AbstractSolver> get_anytime_Apex_solver(AdjacencyMatrix &graph, AdjacencyMatrix &inv_graph,
                                                        size_t source, size_t target, EPS eps, int type                                                   );
