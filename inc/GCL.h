#pragma once

#include "Utils/Definitions.h"
#include <array>
#include <cstddef>
#include <unordered_map>
#include <set>
#include <vector>
#include "GCL_bucket.h"
#include "Utils/common.h"



class G2min {
    
    std::vector<int> g2min;

public:
    std::string get_name(){return "list";}

    G2min(size_t graph_size):g2min(graph_size + 1, MAX_COST) {};

    inline bool is_dominated(size_t state, std::array<cost_t, 1> & gval) {
        return g2min.at(state) <= gval[0];
    }

    inline void add_gval(size_t state, std::array<cost_t, 1> & gval){
        if (g2min.at(state) <= gval[0]){
            cerr << "call GCL with a wrong value" << endl;
            exit(-1);
        }
        g2min.at(state) = gval[0];
    }
};

template<size_t N>
class GCL{
protected:
    std::vector<std::list<std::array<cost_t, N>>> gcl;

public:
    std::string get_name(){return "list";}

    GCL(size_t graph_size):gcl(graph_size + 1) {};

    inline bool is_dominated(size_t state, const std::array<cost_t, N> & gval) {

        if (state >= gcl.size()){
            return false;
        }

        for (auto & vec: gcl[state]){
            if(is_dominating<N>(vec, gval)){
                return true;
            }
        }

        return false;
    }

    inline void add_gval(size_t state, const std::array<cost_t, N> & gval){
        if (state >= gcl.size()){
            cerr << "call GCL with wrong index" << endl;
            exit(-1);
            return;
        }
        auto i = gcl[state].begin();
        while (i != gcl[state].end()) {
            if (is_dominating<N>(gval, *i)){
                gcl[state].erase(i++);  // alternatively, i = items.erase(i);
            }
            else{
                ++i;
            }
        }
        
        gcl[state].push_front(gval);
    }
};


const int ARRAY_DEFAULT_SIZE=50000;

template<size_t N>
class GCL_array{
protected:
    size_t check_cnt=0;
    size_t update_cnt=0;

public:
    std::vector<std::vector<std::array<cost_t, N>>> gcl;


    void reset(){
        size_t size = gcl.size();
        gcl.clear();
        gcl.resize(size);
    }


    std::string get_name(){return "array";}

    GCL_array(size_t graph_size):gcl(graph_size + 1) {
        __is_dominating_called_cnt__ = 0;
        for (int i = 0; i < graph_size; i++){
            gcl[i].reserve(ARRAY_DEFAULT_SIZE);
        }
    };

    inline bool is_dominated_w(size_t state, const std::array<cost_t, N> & gval, std::array<double, N> & w) {
        // check_cnt += 1;
        if (state >= gcl.size()){
            return false;
        }

        for (auto & vec: gcl[state]){
            if(is_dominating<N>(vec, gval, w)){
                return true;
            }
        }

        return false;
    }

    inline bool is_dominated(size_t state, const std::array<cost_t, N> & gval) {
        // check_cnt += 1;
        if (state >= gcl.size()){
            return false;
        }

        for (auto & vec: gcl[state]){
            if(is_dominating<N>(vec, gval)){
                return true;
            }
        }

        return false;
    }

    inline void add_gval(size_t state, const std::array<cost_t, N> & gval){
        // update_cnt += 1;
        if (state >= gcl.size()){
            cerr << "call GCL with wrong index" << endl;
            exit(-1);
            return;
        }

        gcl[state].push_back(gval);


        for (int i = gcl[state].size() - 2 ; i >=0 ;i--){
            if (is_dominating<N>(gval, gcl[state][i])){
                gcl[state][i] = gcl[state].back();
                gcl[state].pop_back();
            }
        }
    }


    inline void add_gval_direct(size_t state, const std::array<cost_t, N> & gval){
        // update_cnt += 1;
        if (state >= gcl.size()){
            cerr << "call GCL with wrong index" << endl;
            exit(-1);
            return;
        }
        gcl[state].push_back(gval);
    }


    ~GCL_array<N>(){
        cout << "vector comparison cnt: " << __is_dominating_called_cnt__ << endl;
    }
};

template<size_t N>
class GCL_array_hash{
protected:
    std::unordered_map<size_t, std::vector<std::array<cost_t, N>>> gcl;

public:
    std::string get_name(){return "array";}

    GCL_array_hash(size_t graph_size=0){
        // for (int i = 0; i < graph_size; i++){
        //     gcl.reserve(ARRAY_DEFAULT_SIZE);
        // }
    };

    inline bool is_dominated(size_t state, std::array<cost_t, N> & gval) {

        if (gcl.find(state) == gcl.end()){
            return false;
        }

        for (auto & vec: gcl[state]){
            if(is_dominating<N>(vec, gval)){
                return true;
            }
        }

        return false;
    }


    inline bool is_dominated(size_t state, std::array<cost_t, N + 1> & gval) {

        if (gcl.find(state) == gcl.end()){
            return false;
        }

        std::array<cost_t, N> gval_t;
    std::copy(std::begin(gval) + 1, std::end(gval), std::begin(gval_t));
        for (auto & vec: gcl[state]){
            if(is_dominating<N>(vec, gval_t)){
                return true;
            }
        }

        return false;
    }

    inline void add_gval(size_t state, std::array<cost_t, N> & gval){

        if (gcl.find(state) == gcl.end()){
            gcl[state] = vector<array<cost_t, N>>();
            gcl[state].reserve(ARRAY_DEFAULT_SIZE);
        }

        gcl[state].push_back(gval);


        for (int i = gcl[state].size() - 2 ; i >=0 ;i--){
            if (is_dominating<N>(gval, gcl[state][i])){
                gcl[state][i] = gcl[state].back();
                gcl[state].pop_back();
            }
      
        }
    }
};
template class GCL_array<3>;
template class GCL_array<4>;
template class GCL_array_hash<2>;
template class GCL_array_hash<3>;
template class GCL_array_hash<4>;


template<size_t N>
class GCL_hash{
protected:
    std::unordered_map<size_t, std::list<std::array<cost_t, N>>> gcl;

public:
    std::string get_name(){return "list";}

    GCL_hash(size_t graph_size=0) {};

    inline bool is_dominated(size_t state, std::array<cost_t, N> & gval) {

        if (gcl.find(state) == gcl.end()){
            return false;
        }

        for (auto & vec: gcl[state]){
            if(is_dominating<N>(vec, gval)){
                return true;
            }
        }

        return false;
    }

    inline void add_gval(size_t state, std::array<cost_t, N> & gval){
        if (gcl.find(state) == gcl.end()){
            gcl[state] = std::list<std::array<cost_t, N>>();
        }
        auto i = gcl[state].begin();
        while (i != gcl[state].end()) {
            if (is_dominating<N>(gval, *i)){
                gcl[state].erase(i++);  // alternatively, i = items.erase(i);
            }
            else{
                ++i;
            }
        }
        
        gcl[state].push_front(gval);
    }
};

class GCL_logtime {

protected:
    std::unordered_map<size_t, std::set<std::pair<cost_t, cost_t>>> gcl;

public:
    std::string get_name() {return "AVLTree";}

    GCL_logtime(size_t graph_size=0) {};

    bool is_dominated(size_t state, std::array<cost_t, 2> & gval);

    void add_gval(size_t state, std::array<cost_t, 2> & gval);
};


template class GCL<2>;
template class GCL<3>;
template class GCL<4>;
template class GCL<5>;
template class GCL<6>;
