#pragma once
#include "Definitions.h"
#include <boost/heap/priority_queue.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <sstream>

using boost::heap::priority_queue;
using boost::heap::pairing_heap;
using boost::heap::compare;

template <int N>
struct ApexPathPairWHandles: public ApexPathPair<N> {
    std::array<long double, N> key;
    bool in_focal = false;

    struct compare_open
    {
        bool operator()(ApexPathPairWHandles<N>* const &a, ApexPathPairWHandles<N>* const &b) const;
    };

    struct compare_focal
    {
        bool operator()(ApexPathPairWHandles<N>* const &a, ApexPathPairWHandles<N>* const &b) const;
    };

    typename pairing_heap<ApexPathPairWHandles<N>*, compare<ApexPathPairWHandles<N>::compare_open> >::handle_type open_handle;
    typename pairing_heap<ApexPathPairWHandles<N>*, compare<ApexPathPairWHandles<N>::compare_focal> >::handle_type focal_handle;


    ApexPathPairWHandles(){}
    ApexPathPairWHandles(size_t id, std::array<cost_t, N> f,
                          std::array<cost_t, N> path_cost,
                          ApexPathPair<N>* parent = nullptr):
        ApexPathPair<N>(id, f, path_cost, parent)
    {
    };


    friend std::ostream &operator<<(std::ostream &stream, const ApexPathPairWHandles &ap);


    std::string to_str(){
        std::stringstream ss;
        ss<< "f= " << ApexPathPair<N>::f << ", path_cost= " << ApexPathPair<N>::path_cost << "key= " ;
        for (int i = 0; i < N; i++){
            ss << key[i] << ", ";
        }
        return ss.str();
    }


};
