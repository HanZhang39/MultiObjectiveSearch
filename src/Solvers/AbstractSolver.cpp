#include "AbstractSolver.h"
#include <boost/unordered_set.hpp>

std::ostream& operator <<(std::ostream &stream, const std::vector<double> &vec){
    stream << "[";
    for (size_t i = 0 ;  i < vec.size(); i ++){
        stream << vec[i];
        if (i + 1 <vec.size()){
            stream << ", ";
        }
    }
    stream << "]";
    return stream;
}

std::string AbstractSolver::get_result_str(){
    std::stringstream ss;
    ss << get_solver_name() <<  "\t"
       << source << "\t" << target << "\t"
       << get_num_generation() << "\t"
       << get_num_expansion() << "\t"
       << solutions.size() << "\t"
        ;
    std::string result = ss.str();
    return result;
}
