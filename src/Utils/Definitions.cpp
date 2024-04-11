#include <ctime>
#include <iostream>
#include <set>
#include <string>
#include "Utils/Definitions.h"


template <int N>
bool Node<N>::more_than_specific_heuristic_cost::operator()(const NodePtr<N> &a, const NodePtr<N> &b) const {
  return (a->f[cost_idx] > b->f[cost_idx]);
}


template <int N>
bool Node<N>::more_than_full_cost::operator()(const NodePtr<N> &a, const NodePtr<N> &b) const {
  for (int i = 0; i + 1 < N; i++){
    if (a->f[i] != b->f[i]) {
      return (a->f[i] > b->f[i]);
    }
  }
  return (a->f.back() > b->f.back());
}

template <int N>
bool Node<N>::more_than_tr_cost::operator()(const NodePtr<N> &a, const NodePtr<N> &b) const {
  for (int i = 1; i + 1 < N; i++){
    if (a->f[i] != b->f[i]) {
      return (a->f[i] > b->f[i]);
    }
  }
  return (a->f.back() > b->f.back());
}
// std::ostream& operator <<(std::ostream &stream, const std::vector<size_t> &vec){
//   stream << "[";
//   for (size_t i = 0 ;  i < vec.size(); i ++){
//     stream << vec[i];
//     if (i + 1 <vec.size()){
//       stream << ", ";
//     }
//   }
//   stream << "]";
//   return stream;
// }


template class Node<2>;
template class Node<3>;
template class Node<4>;
template class Node<5>;
template class Node<6>;

std::ostream &operator<<(std::ostream &stream, const CostVec<2> &cost_vec){
    stream << "[";
    stream << cost_vec[0] << ", ";
    stream << cost_vec[1];
    stream << "]";
    return stream;
}


std::ostream &operator<<(std::ostream &stream, const CostVec<3> &cost_vec){
  stream << "[";
  stream << cost_vec[0] << ", ";
  stream << cost_vec[1] << ", ";
  stream << cost_vec[2];
  stream << "]";
  return stream;
}

std::ostream &operator<<(std::ostream &stream, const CostVec<4> &cost_vec){
  stream << "[";
  stream << cost_vec[0] << ", ";
  stream << cost_vec[1] << ", ";
  stream << cost_vec[2] << ", ";
  stream << cost_vec[3];
  stream << "]";
  return stream;
}

std::ostream &operator<<(std::ostream &stream, const CostVec<5> &cost_vec){
  stream << "[";
  stream << cost_vec[0] << ", ";
  stream << cost_vec[1] << ", ";
  stream << cost_vec[2] << ", ";
  stream << cost_vec[3] << ", ";
  stream << cost_vec[4];
  stream << "]";
  return stream;
}


std::ostream &operator<<(std::ostream &stream, const CostVec<6> &cost_vec){
  stream << "[";
  stream << cost_vec[0] << ", ";
  stream << cost_vec[1] << ", ";
  stream << cost_vec[2] << ", ";
  stream << cost_vec[3] << ", ";
  stream << cost_vec[4] << ", ";
  stream << cost_vec[5];
  stream << "]";
  return stream;
}

std::ostream &operator<<(std::ostream &stream, const Solution & sol){
    stream << "[";
    if (sol.apex.size() > 0){
        stream << sol.apex.front();
    }
    for (int i = 1; i < sol.apex.size(); i++){
        stream << "," << sol.apex[i];
    }
    stream << "],";

    stream << "[";
    if (sol.cost.size() > 0){
        stream << sol.cost.front();
    }
    for (int i = 1; i < sol.cost.size(); i++){
        stream << "," << sol.cost[i];
    }
    stream << "],";

    stream <<  (double)sol.time_found / CLOCKS_PER_SEC;
    return stream;
}


unsigned long long __is_dominating_called_cnt__ = 0;

