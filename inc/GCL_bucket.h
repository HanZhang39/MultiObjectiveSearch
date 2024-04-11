#pragma once
#include "Utils/Definitions.h"
#include "Utils/pool.h"

extern int bucket_step;

template <size_t N>
struct Bucket {

    CostVec<N> index;

    std::vector<std::array<cost_t, N>> gcl;

    bool _dominating_(std::array<cost_t, N> & gval);

    inline bool dominating(std::array<cost_t, N> & gval);


    void prune(std::array<cost_t, N> & gval);

    bool prune_and_update(std::array<cost_t, N> & gval);

    Bucket<N>* next_ = nullptr; // used in list

    Bucket(CostVec<N> & gval);
    Bucket();

    void update(CostVec<N> & gval) ;

    inline Bucket<N>* get_next() { return next_; }

    inline void set_next(Bucket<N>* next) { next_ = next; }
};

template<size_t N>
class GCL_array_bucket{
protected:
    pool<Bucket<N>> bucket_pool = pool<Bucket<N>>(500000);
    std::vector<std::vector<Bucket<N>*>> gcl;
    size_t check_cnt=0;
    size_t update_cnt=0;

public:
    std::string get_name(){return "array_bucket";}

    GCL_array_bucket(size_t graph_size):gcl(graph_size + 1) {
        for (int i = 0; i < graph_size; i++){
            gcl[i].reserve(500);
        }
        __is_dominating_called_cnt__ = 0;
    };

    inline bool is_dominated(size_t state, std::array<cost_t, N> & gval) {
        if (state >= gcl.size()){
            return false;
        }

        for (auto bucket: gcl[state]){
            if(bucket->dominating(gval)){
                return true;
            }
        }

        return false;
    }

    inline void add_gval(size_t state, std::array<cost_t, N> & gval){
        // update_cnt += 1;
        if (state >= gcl.size()){
            cerr << "call GCL with wrong index" << endl;
            exit(-1);
            return;
        }

        bool added = false;
        for (int i = gcl[state].size() - 1 ; i >=0 ;i--){

            added = added || gcl[state][i]->prune_and_update(gval);
            if (gcl[state][i]->gcl.empty()){
                if (i==gcl[state].size() - 1){
                    // delete gcl[state][i];
                    bucket_pool.save_label(gcl[state][i]);
                    gcl[state].pop_back();
                } else {
                    // delete gcl[state][i];
                    bucket_pool.save_label(gcl[state][i]);
                    gcl[state][i] = gcl[state].back();
                    gcl[state].pop_back();
                }
            }
        }

        if (!added){
            Bucket<N> *bucket_ptr = bucket_pool.get_label();
            bucket_ptr->update(gval);
            gcl[state].push_back(bucket_ptr);
        }

        // gcl[state].push_back(gval);
    }

    ~GCL_array_bucket<N>(){
        // for (auto & l: gcl){
        //     for (Bucket<N>* buck: l){
        //         delete buck;
        //     }
        // }
        cout << "vector comparison cnt: " << __is_dominating_called_cnt__ << endl;
    }

};

template  class GCL_array_bucket<2>;
template  class GCL_array_bucket<3>;
template  class GCL_array_bucket<4>;
template  class GCL_array_bucket<5>;
