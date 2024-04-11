                                                \
#pragma once

#include <cstddef>
#include <map>
#include <vector>
#include <array>
#include <list>
#include <iostream>
#include <limits>
#include <functional>
#include <memory>
#include <climits>
#include "boost/heap/pairing_heap.hpp"
#include "boost/heap/priority_queue.hpp"
#include "common.h"
#include "Graph.h"


#ifndef DEBUG
#define DEBUG 1
#endif


const cost_t MAX_COST = std::numeric_limits<cost_t>::max();

template<int N>
using Heuristic = std::function<std::array<cost_t, N>&(size_t)>;

enum BO_LEX_ORDER {LEX0, LEX1};


template <int N>
struct Node;

template <int N> using NodePtr = Node<N> *;

typedef Node<2> Node2;
typedef NodePtr<2> NodePtr2;

template <int N>
struct Node{
    static const int num_obj=N;

    size_t id;
    std::array<cost_t, N> f;

    size_t parent;
    Node* next_=nullptr;
    bool should_check_sol=false;

    inline Node *
    get_next()
    {
        return next_;
    }

    inline void
    set_next(Node *next)
    {
        next_ = next;
    }

    Node(size_t id, std::array<cost_t, N> f)
        : id(id),
          f(f)
    {
    };

    Node()
    {
    };

    Node(size_t id, std::array<cost_t, N> g,
         std::array<cost_t, N> &  h
        )
        : id(id),
          f(g)
    {
        for (int i = 0; i < N; i++){
            f[i] = g[i] + h[i];
        }
    };

    struct more_than_specific_heuristic_cost {
        size_t cost_idx;

        more_than_specific_heuristic_cost(size_t cost_idx) : cost_idx(cost_idx) {};
        bool operator()(const NodePtr<N> &a, const NodePtr<N> &b) const;
    };

    struct more_than_full_cost {
        bool operator()(const NodePtr<N> &a, const NodePtr<N> &b) const;
    };

    struct more_than_tr_cost {
        inline bool operator()(const NodePtr<N> &a, const NodePtr<N> &b) const;
    };

};

struct Solution {
    vector<cost_t> apex;
    vector<cost_t> cost;

    double time_found;

    Solution(vector<cost_t> _apex = {}, vector<cost_t> _cost = {}, double time=-1):
        apex(_apex), time_found(time) {
        if (_cost.size() == 0){
            cost = _apex;
        } else {
            cost = _cost ;
        }
    }
};


std::ostream &operator<<(std::ostream &stream, const Solution &sol);
 
using SolutionSet = std::vector<Solution>;

template <typename T> using Pair = std::array<T, 2>;

template <int N> using CostVec = std::array<cost_t, N>;

// TODO make it template
// template <int N>
// std::ostream &operator<<(std::ostream &stream, const CostVec<N> &cost_vec);


std::ostream &operator<<(std::ostream &stream, const CostVec<2> &cost_vec);
std::ostream &operator<<(std::ostream &stream, const CostVec<3> &cost_vec);
std::ostream &operator<<(std::ostream &stream, const CostVec<4> &cost_vec);
std::ostream &operator<<(std::ostream &stream, const CostVec<5> &cost_vec);
std::ostream &operator<<(std::ostream &stream, const CostVec<6> &cost_vec);

template <int N>
inline CostVec<N> min(const CostVec<N> & vec_a, const CostVec<N> & vec_b){
  CostVec<N> res = vec_a;

  for (size_t i = 0; i < N; i++) {
    if (res[i] > vec_b[i]) {
      res[i] = vec_b[i];
    }
  }
  return res;
}

template <int N>
inline std::array<long double, N> min(const std::array<long double, N> & vec_a, const std::array<long double, N> & vec_b){
  std::array<long double, N> res = vec_a;

  for (size_t i = 0; i < N; i++) {
    if (res[i] > vec_b[i]) {
      res[i] = vec_b[i];
    }
  }
  return res;
}


template <int N>
inline CostVec<N - 1> Tr(const CostVec<N> & vec){
    CostVec<N - 1> tr_vec;
    std::copy(std::begin(vec) + 1, std::end(vec), std::begin(tr_vec));
    return tr_vec;
}

template <int N>
inline CostVec<N> convert(const vector<cost_t> & vec){
    CostVec<N> res_vec;
    std::copy(std::begin(vec), std::end(vec), std::begin(res_vec));
    return res_vec;
}

template <int N>
inline CostVec<N> sum(const CostVec<N> &vec_a, const CostVec<N> &vec_b) {
  CostVec<N> res = vec_a;

  for (size_t i = 0; i < N; i++) {
      res[i] = res[i] + vec_b[i];
  }
  return res;
}

using EPS = std::vector<double>;


template <int N>
struct ApexPathPair;

template <int N> using ApexPathPairPtr = ApexPathPair<N> *;

template <int N>
struct ApexPathPair {
    size_t id; // state of the node
    std::array<cost_t, N> f;
    std::array<cost_t, N> path_cost;
    // ApexPathPair* parent;
    bool active=true;

    ApexPathPair* parent = nullptr;

    ApexPathPair* next_ = nullptr; // used in list
    ApexPathPair *next_in_map_ = nullptr; // used in map
    ApexPathPair *prev_in_map_ = nullptr; // used in map

    ApexPathPair(){}
    ApexPathPair(size_t id, std::array<cost_t, N> f,
                 std::array<cost_t, N> path_cost,
                 ApexPathPair* parent = nullptr)
        : id(id), f(f), path_cost(path_cost), active(true)
        , parent(parent)
    {
    };


    struct more_than_full_cost {
        bool operator()(const ApexPathPairPtr<N> &a, const ApexPathPairPtr<N> &b) const;
    };

    friend std::ostream &operator<<(std::ostream &stream, const ApexPathPair &ap);

    inline ApexPathPair* get_next() { return next_; }

    inline void set_next(ApexPathPair* next) { next_ = next; }
};


std::ostream& operator<<(std::ostream& os, const std::vector<cost_t>& vec);


extern unsigned long long __is_dominating_called_cnt__;

template <unsigned int N>
inline int dominate_relation(const CostVec<N> & apex, const std::array<cost_t, N> & node){
    __is_dominating_called_cnt__ += 1;
    bool dominate = true;
    bool dominated = true;
    for (int i = 0; i < N && (dominate || dominated); i ++ ){
        if (node[i] < apex[i]){
            dominate =false;
        }
        if (node[i] > apex[i]){
            dominated = false;
        }
    }
    return (dominated << 1) + (int)dominate;
}


template <unsigned int N>
inline bool is_dominating(const CostVec<N> & apex, const std::array<cost_t, N> & node){
    // return dominate_relation<N>(apex, node) & 1;
    __is_dominating_called_cnt__ += 1;
    for (int i = 0; i < N; i ++ ){
        if (node[i] < apex[i]){
            return false;
        }
    }
    return true;
}


template <unsigned int N>
inline bool is_dominating(const CostVec<N> & apex, const std::array<cost_t, N> & node, std::array<double, N> & w){
    __is_dominating_called_cnt__ += 1;
    for (int i = 0; i < N; i ++ ){
        if ((double)node[i]  * w[i]< (double)apex[i]){
            return false;
        }
    }
    return true;
}

template <unsigned int N>
bool is_dominating_dr(const std::array<cost_t, N> & apex, const std::array<cost_t, N> & node){
    for (int i = 1; i < N; i ++ ){
        if (node[i] < apex[i]){
            return false;
        }
    }
    return true;
}

template <unsigned int N>
bool is_dominating_dr(const std::array<cost_t, N> & path_cost, const std::array<cost_t, N> & f_val, const vector<double> & eps){
    for (int i = 1; i < N; i ++ ){
        if (path_cost[i] > (1 + eps.at(i)) * f_val[i]){
            return false;
        }
    }
    return true;
}


class PQ:public boost::heap::priority_queue<NodePtr2, boost::heap::compare<Node2::more_than_full_cost>>{
public:
    PQ(int width=0, int lb=0, int ub=0){}
};
