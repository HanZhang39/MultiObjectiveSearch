#pragma once

#include <cstddef>
#include <memory>
#include <vector>
#include "Utils/Definitions.h"
#include "ShortestPathHeuristic.h"
#include "AbstractSolver.h"
#include "GCL_bucket.h"
#include "LTMOA.h"

template<size_t N>
class GCL_table_bucket{
protected:
    pool<Bucket<N>> bucket_pool = pool<Bucket<N>>(500000);
    size_t check_cnt=0;
    size_t update_cnt=0;

public:
    std::vector<std::vector<std::pair<size_t, std::vector<Bucket<N>*>>>> gcl;

    GCL_table_bucket(size_t graph_size):gcl(graph_size + 1) {
        __is_dominating_called_cnt__ = 0;
        for (auto& l: gcl){
            l.reserve(5);
        }
    };

    inline bool is_dominated(size_t state, size_t parent, std::array<cost_t, N> & gval) {
        if (state >= gcl.size()){
            return false;
        }

        for (auto & id_vec_pair: gcl[state]){
            if (id_vec_pair.first == parent){continue;}

            for (auto bucket: id_vec_pair.second){
                if( bucket->dominating(gval)){
                    return true;
                }
            }
        }

        return false;
    }

    inline void add_gval(size_t state, size_t parent, std::array<cost_t, N> & gval){
        // update_cnt += 1;
        if (state >= gcl.size()){
            cerr << "call GCL with wrong index" << endl;
            exit(-1);
            return;
        }
        bool added = false;
        for (auto & id_vec_pair: gcl[state]){
            if (id_vec_pair.first == parent){

                for (int i = id_vec_pair.second.size() - 1 ; i >=0 ;i--){
                    added = added || id_vec_pair.second[i]->prune_and_update(gval);
                    if (id_vec_pair.second[i]->gcl.empty()){
                        if (i==id_vec_pair.second.size() - 1){
                            // delete id_vec_pair.second[i];
                            bucket_pool.save_label(id_vec_pair.second[i]);
                            id_vec_pair.second.pop_back();
                        } else {
                            // delete id_vec_pair.second[i];
                            bucket_pool.save_label(id_vec_pair.second[i]);
                            id_vec_pair.second[i] = id_vec_pair.second.back();
                            id_vec_pair.second.pop_back();
                        }
                    }
                }

                if (!added){
                    Bucket<N> *bucket_ptr = bucket_pool.get_label();
                    bucket_ptr->update(gval);
                    id_vec_pair.second.push_back(bucket_ptr);
                    added = true;
                }


            } else {

                for (int i = id_vec_pair.second.size() - 1 ; i >=0 ;i--){
                    id_vec_pair.second[i]->prune(gval);
                    if (id_vec_pair.second[i]->gcl.empty()){
                        if (i==id_vec_pair.second.size() - 1){
                            // delete id_vec_pair.second[i];
                            bucket_pool.save_label(id_vec_pair.second[i]);
                            id_vec_pair.second.pop_back();
                        } else {
                            // delete id_vec_pair.second[i];
                            bucket_pool.save_label(id_vec_pair.second[i]);
                            id_vec_pair.second[i] = id_vec_pair.second.back();
                            id_vec_pair.second.pop_back();
                        }
                    }
                }
            }
        }

        if (!added){
            gcl[state].push_back({parent, {}});

            Bucket<N> *bucket_ptr = bucket_pool.get_label();
            bucket_ptr->update(gval);
            gcl[state].back().second.push_back(bucket_ptr);
        }
    }




    ~GCL_table_bucket<N>(){
        cout << "vector comparison cnt: " << __is_dominating_called_cnt__ << endl;
    }
};

template class GCL_table_bucket<3>;


template<size_t N>
class GCL_table{
protected:
    size_t check_cnt=0;
    size_t update_cnt=0;

public:
    std::vector<std::list<std::pair<size_t, std::vector<std::array<cost_t, N>>>>> gcl;

    GCL_table(size_t graph_size):gcl(graph_size + 1) {
        __is_dominating_called_cnt__ = 0;
    };

    inline bool is_dominated(size_t state, size_t parent, const std::array<cost_t, N> & gval) {
        // check_cnt += 1;
        if (state >= gcl.size()){
            return false;
        }

        for (auto & id_vec_pair: gcl[state]){
            if (id_vec_pair.first == parent){continue;}

            for (auto & vec: id_vec_pair.second){
                if(is_dominating<N>(vec, gval)){
                    return true;
                }
            }
        }

        return false;
    }

    inline void add_gval(size_t state, size_t parent, const std::array<cost_t, N> & gval){
        // update_cnt += 1;
        if (state >= gcl.size()){
            cerr << "call GCL with wrong index" << endl;
            exit(-1);
            return;
        }
        bool added = false;
        for (auto & id_vec_pair: gcl[state]){
            if (id_vec_pair.first == parent){
                added = true;
                id_vec_pair.second.push_back(gval);

                for (int i = id_vec_pair.second.size() - 2 ; i >=0 ;i--){
                    if (is_dominating<N>(gval, id_vec_pair.second[i])){
                        id_vec_pair.second[i] = id_vec_pair.second.back();
                        id_vec_pair.second.pop_back();
                    }
                }
            } else {
                while (id_vec_pair.second.size() > 0 &&
                       is_dominating<N>(gval, id_vec_pair.second.back())
                       ){
                    id_vec_pair.second.pop_back();
                }
                for (int i = id_vec_pair.second.size() - 2 ; i >=0 ;i--){
                    if (is_dominating<N>(gval, id_vec_pair.second[i])){
                        id_vec_pair.second[i] = id_vec_pair.second.back();
                        id_vec_pair.second.pop_back();
                    }
                }
            }
        }

        if (!added){
            gcl[state].push_back({parent, {gval}});
        }
    }




    ~GCL_table<N>(){
        cout << "vector comparison cnt: " << __is_dominating_called_cnt__ << endl;
    }
};

template <int N, class GCL>
class LTMOA2: public AbstractSolver {
protected:

    std::list<Node<N>*> nodes;
    GCL gcl;

    virtual bool is_dominated(Node<N>* node);
    virtual bool is_dominated(size_t id, size_t parent_id, bool check_sol, std::array<cost_t, N> f_val);
    virtual bool is_dominated(size_t id, size_t parent_id, bool check_sol, std::array<cost_t, N - 1> tr_f);

    std::clock_t queue_time = 0;
    std::clock_t dominance_check_time = 0;

public:


    ShortestPathHeuristic<N> heuristic;
    LTMOA2(AdjacencyMatrix &adj_matrix, AdjacencyMatrix &inv_graph, size_t source, size_t target, EPS eps):
        AbstractSolver(adj_matrix, source, target, eps),
        gcl(adj_matrix.size()),
        heuristic(ShortestPathHeuristic<N>(target, inv_graph))
    {}

    virtual std::string get_solver_name() override {return "LTMOA2(" + std::to_string(N)+")-"; }

    void solve(unsigned int time_limit=UINT_MAX) override;

    virtual ~LTMOA2(){
        for (auto ptr:nodes){
            delete ptr;
        }
    }

};

std::shared_ptr<AbstractSolver>
get_LTMOA2_solver(AdjacencyMatrix &graph, AdjacencyMatrix &inv_graph,
                   size_t source, size_t target,
                    bool use_bucket);
