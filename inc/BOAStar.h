#pragma once

#include <cstddef>
#include <vector>
#include "Utils/Definitions.h"
#include "AbstractSolver.h"
#include "ShortestPathHeuristic.h"
#include "Utils/pqueue_bucket.h"


template <class Q>
class BOAStar: public AbstractSolver {
protected:
    ShortestPathHeuristic<2> heuristic;

public:

    virtual std::string get_solver_name() {return "BOA*"; }

    BOAStar(AdjacencyMatrix &graph, AdjacencyMatrix & inv_graph, size_t source, size_t target);

    void solve(unsigned int time_limit=UINT_MAX);

};



// using BOAStarBucket = BOAStar<pqueue_label_bucket<Node<2>>>;
using BOAStarBucket = BOAStar<pqueue_label_bucket<Node<2>>>;
using BOAStarPQ = BOAStar<PQ>;
