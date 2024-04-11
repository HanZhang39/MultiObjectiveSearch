#pragma once
#include <vector>
#include "ShortestPathHeuristic.h"
#include "Utils/Definitions.h"


class AStar {
private:
  const AdjacencyMatrix   &adj_matrix;

public:
  AStar(const AdjacencyMatrix &adj_matrix);
  NodePtr<2> operator()(size_t source, size_t target, ShortestPathHeuristic<2> &heuristic, BO_LEX_ORDER order = BO_LEX_ORDER::LEX0);
};
