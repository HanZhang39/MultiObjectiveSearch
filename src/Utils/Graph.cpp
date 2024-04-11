#include "Utils/Definitions.h"


AdjacencyMatrix::AdjacencyMatrix(size_t graph_size, std::vector<Edge> &edges,
                                 bool inverse)
    : matrix((graph_size), std::vector<Edge>()),
      num_of_objectives(edges[0].cost.size()) {
  for (auto iter = edges.begin(); iter != edges.end(); ++iter) {
    if (iter->source >= graph_size || iter->target >= graph_size) {
      std::cerr << "inconsistent graph size";
      exit(1);
    }

    if (inverse) {
      this->add(iter->inverse());
    } else {
      this->add((*iter));
    }
  }
}


size_t AdjacencyMatrix::get_num_of_objectives() const {
  return num_of_objectives;
}

void AdjacencyMatrix::add(Edge edge) {
    (this->matrix[edge.source]).push_back(edge);
}

size_t AdjacencyMatrix::size() const { return this->matrix.size(); }
 

std::vector<Edge>& AdjacencyMatrix::operator[](size_t vertex_id) {
    return this->matrix.at(vertex_id);
}

const std::vector<Edge>& AdjacencyMatrix::operator[](size_t vertex_id) const {
  return this->matrix.at(vertex_id);
}


std::ostream& operator<<(std::ostream &stream, const AdjacencyMatrix &adj_matrix) {
    size_t  i = 0;

    stream << "{\n";
    for (auto vertex_iter = adj_matrix.matrix.begin(); vertex_iter != adj_matrix.matrix.end(); ++vertex_iter) {
      stream << "\t\"" << i++ << "\": [";

      std::vector<Edge> edges = *vertex_iter;
      for (auto edge_iter = edges.begin(); edge_iter != edges.end(); ++edge_iter) {
        stream << "\"" << edge_iter->source << "->" << edge_iter->target << "\", ";
      }

      stream << "],\n";
    }
    stream << "}";
    return stream;
}

std::ostream& operator<<(std::ostream &stream, const Edge &edge) {
  // Printed in JSON format
  stream
    << "{"
    <<  "\"edge_source\": " << edge.source << ", "
    <<  "\"edge_target\": " << edge.target << ", "
    <<  "\"edge_cost\": ";

  for (auto c: edge.cost){
    stream << c << ", ";
  }
  stream << "}";

  return stream;
}

void MultiCostEdge::insert(std::vector<cost_t> cost) {
    costs.push_back(cost);
    if (min_cost.size() == 0){
        min_cost = cost;
        return;
    }

    for (int i = 0; i < min_cost.size(); i++){
        min_cost[i] = min(cost[i], min_cost[i]);
    }
};


void MultiCostEdge::insert(const Edge & edge) {
    costs.push_back({edge.apex, edge.cost});
    if (min_cost.size() == 0){
        min_cost = edge.apex;
        return;
    }

    for (int i = 0; i < min_cost.size(); i++){
        min_cost[i] = min(edge.apex[i], min_cost[i]);
    }
};

AdjacencyMultiCostMatrix::AdjacencyMultiCostMatrix(size_t graph_size,
                                                   std::vector<Edge> &edges)
    :matrix(graph_size), inv_matrix(graph_size), num_of_objectives(edges[0].cost.size())
{
    for (auto iter = edges.begin(); iter != edges.end(); ++iter) {
        if (iter->source >= graph_size || iter->target >= graph_size) {
            std::cerr << "inconsistent graph size";
            exit(1);
        }

        this->add((*iter));
    }
}

void AdjacencyMultiCostMatrix::add(const Edge & edge){
    for (shared_ptr<MultiCostEdge> & ptr: matrix[edge.source]){
        if (ptr->target == edge.target){
            ptr->insert(edge);
            return;
        }
    }

    // create new multi-cost-edge

    shared_ptr<MultiCostEdge> new_edge = std::make_shared<MultiCostEdge>();
    new_edge->source = edge.source;
    new_edge->target = edge.target;
    new_edge->insert(edge);

    matrix[edge.source].push_back(new_edge);
    inv_matrix[edge.target].push_back(new_edge);
}

void AdjacencyMultiCostMatrix::add(const MultiCostEdge & edge){
    for (shared_ptr<MultiCostEdge> & ptr: matrix[edge.source]){
        if (ptr->target == edge.target){
            cout << "exist multi-cost edge" << endl;
            ptr->costs.insert(ptr->costs.end(), edge.costs.begin(), edge.costs.end());

            return;
        }
    }

    // create new multi-cost-edge
    shared_ptr<MultiCostEdge> new_edge = std::make_shared<MultiCostEdge>();
    *new_edge = edge;

    matrix[edge.source].push_back(new_edge);
    inv_matrix[edge.target].push_back(new_edge);
}
