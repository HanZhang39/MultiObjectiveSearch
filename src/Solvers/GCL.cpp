#include "GCL.h"


bool is_dominated_(std::set<std::pair<cost_t, cost_t>>& pair_set, std::pair<cost_t, cost_t> vec){
    if ( pair_set.size() == 0 ){return false;}
    auto it = pair_set.lower_bound(vec);
    if (it != pair_set.end() && it->first == vec.first && it->second == vec.second){
        return true;
    }
    if (it != pair_set.begin()){
        auto it_prev = std::prev(it);
        if (it_prev->second <= vec.second){
            return true;
        }
    }
    return false;
}

bool is_dom(const std::pair<cost_t, cost_t>& p1, const std::pair<cost_t, cost_t>& p2){
    return p1.first <= p2.first && p1.second <= p2.second;
};

void remove_dominated(std::set<std::pair<cost_t,cost_t>> & pair_set, std::pair<cost_t,cost_t> p){
    auto it = pair_set.upper_bound(p);
    if (it != pair_set.end()){
        auto curr = it;
        while (curr != pair_set.end() && is_dom(p, *curr)){
            curr = next(curr);
        }
        pair_set.erase(it, curr);
    }
}

bool GCL_logtime::is_dominated(size_t state, std::array<cost_t, 2> & gval){
    if (gcl.find(state) == gcl.end()){
        return false;
    }
    return is_dominated_(gcl[state], {gval[0], gval[1]});
}

void GCL_logtime::add_gval(size_t state, std::array<cost_t, 2> & gval){
    if (gcl.find(state) == gcl.end()){
        gcl[state] = std::set<std::pair<cost_t, cost_t>>();
        gcl[state].insert({gval[0], gval[1]});
        return;
    }

    remove_dominated(gcl[state], {gval[0], gval[1]});
    gcl[state].insert({gval[0], gval[1]});

    return;
}
