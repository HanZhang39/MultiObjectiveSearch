#include "Utils/Definitions.h"
#include <random>

// return true if node dom ape x

bool is_dominating_dr(std::vector<cost_t> & apex, std::vector<cost_t> & node){
  for (int i = 1; i < apex.size(); i ++ ){
    if (node[i] < apex[i]){
      return false;
    }
  }
  return true;
}

bool is_dominating(std::vector<cost_t> & apex, std::vector<cost_t> & node){
  for (int i = 0; i < apex.size(); i ++ ){
    if (node[i] < apex[i]){
      return false;
    }
  }
  return true;
}



bool is_eps_dominating(std::vector<cost_t> & apex, std::vector<cost_t> & node, const EPS eps){
  for (int i = 0; i < apex.size(); i ++ ){
    if ((1 + eps[i]) * node[i] <= apex[i]){
      return false;
    }
  }
  return true;
}

bool is_dominating(std::vector<cost_t> & apex, std::vector<cost_t> & node, std::vector<cost_t> & h){
  for (int i = 0; i < apex.size(); i ++ ){
    if (node[i] + h[i] <= apex[i]){
      return false;
    }
  }
  return true;
}



bool is_bounded(std::vector<cost_t> & apex, std::vector<cost_t> & pathcost, std::vector<cost_t> & h, const EPS eps){
  for (int i = 0; i < apex.size(); i ++ ){
    if (pathcost[i] + h[i] > (1 + eps[i]) * (apex[i] + h[i]) ){
      return false;
    }
  }
  return true;
}

double compute_slack(std::vector<cost_t> & apex, std::vector<cost_t> & path_cost,  const EPS eps){
  double min_slack = ( (1 + eps[0]) - (double)path_cost[0] / (double) apex[0] ) / eps[0];
  for (int i = 1; i < apex.size(); i ++ ){

    double slack = ( (1 + eps[i]) - (double)path_cost[i] / (double) apex[i] ) / eps[i];
    if (slack < min_slack){
      min_slack = slack;
    }
  }
  return min_slack;
}

template <int N>
bool ApexPathPair<N>::more_than_full_cost::operator()(const ApexPathPairPtr<N> &a, const ApexPathPairPtr<N> &b) const {
  for (int i = 0; i + 1 < N; i++){
    if (a->f[i] != b->f[i]) {
      return (a->f[i] > b->f[i]);
    }
  }
  return (a->f.back() > b->f.back());
}


std::ostream& operator<<(std::ostream& os, const std::vector<cost_t>& vec){
  for (size_t i = 0; i < vec.size(); i++){
    if (i!= 0){
      os << ", ";
    }
    os << vec[i];
  }
  return os;
}

template <int N>
std::ostream& operator<<(std::ostream &stream, const ApexPathPair<N> &ap) {
  // Printed in JSON format
  stream << "g: " << (ap.f) << " path cost:" << (ap.path_cost) << "}";
  return stream;
}

template class ApexPathPair<2>;
template class ApexPathPair<3>;
template class ApexPathPair<4>;
template class ApexPathPair<5>;
template class ApexPathPair<6>;
