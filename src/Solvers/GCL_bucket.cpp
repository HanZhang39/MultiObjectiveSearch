#include "GCL_bucket.h"

int bucket_step = 20000;

template <size_t N>
inline bool Bucket<N>::_dominating_(std::array<cost_t, N> & gval){
    for (auto & val: gcl){
        if (is_dominating<N>(val, gval)){
            return true;
        }
    }
    return false;
}

template <size_t N>
inline bool Bucket<N>::dominating(std::array<cost_t, N> & gval){
    if (gcl.size() <= 1 || is_dominating<N>(index, gval)){
        for (const auto & val: gcl){
            if (is_dominating<N>(val, gval)){
                return true;
            }
        }
    }
    return false;
}

template <size_t N>
void Bucket<N>::prune(std::array<cost_t, N> & gval){
    bool equal = true;
    bool strict_dom = true;
    bool weak_dom = true;
    for (int i = 0; i< N; i++){
        int tmp_index = gval[i]/bucket_step * bucket_step;
        if (index[i] != tmp_index){
            equal = false;
        }
        if (index[i] <= tmp_index){
            strict_dom = false;
        }
        if (index[i] < tmp_index){
            weak_dom = false;
        }
    }

    if (strict_dom){
        gcl.clear();
        return;
    }

    if (weak_dom){
        for (int i = gcl.size() - 1; i >=0 ;i--){
            if (is_dominating<N>(gval, gcl[i])){
                if (i == gcl.size() - 1){
                    gcl.pop_back();
                } else {
                    gcl[i] = gcl.back();
                    gcl.pop_back();
                }
            }
        }
    }

    return;
}

template <size_t N>
bool Bucket<N>::prune_and_update(std::array<cost_t, N> & gval){
    bool equal = true;
    bool strict_dom = true;
    bool weak_dom = true;
    for (int i = 0; i< N; i++){
        int tmp_index = gval[i]/bucket_step * bucket_step;
        if (index[i] != tmp_index){
            equal = false;
        }
        if (index[i] <= tmp_index){
            strict_dom = false;
        }
        if (index[i] < tmp_index){
            weak_dom = false;
        }
    }

    if (strict_dom){
        gcl.clear();
        return false;
    }

    if (weak_dom){
        for (int i = gcl.size() - 1; i >=0 ;i--){
            if (is_dominating<N>(gval, gcl[i])){
                if (i == gcl.size() - 1){
                    gcl.pop_back();
                } else {
                    gcl[i] = gcl.back();
                    gcl.pop_back();
                }
            }
        }
    }

    if (equal){
        gcl.push_back(gval);
    }

    return equal;
}

template <size_t N>
Bucket<N>::Bucket(){
    gcl.reserve(100);
}


template <size_t N>
Bucket<N>::Bucket(CostVec<N> & gval){
    gcl.reserve(10);
    gcl.push_back(gval);
    for (int i = 0 ; i <N;i++){
        index[i] = gval[i]/bucket_step * bucket_step;
    }
}

template <size_t N>
void Bucket<N>::update(CostVec<N> & gval){
    gcl.push_back(gval);
    for (int i = 0 ; i <N;i++){
        index[i] = gval[i]/bucket_step * bucket_step;
    }
}

template class Bucket<1>;
template class Bucket<2>;
template class Bucket<3>;
template class Bucket<4>;
template class Bucket<5>;
