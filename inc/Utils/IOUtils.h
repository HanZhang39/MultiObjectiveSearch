#ifndef UTILS_IO_UTILS_H
#define UTILS_IO_UTILS_H

#include <iostream>
#include <cstring>
#include <string>
#include <vector>
#include "Utils/Definitions.h"

bool load_gr_files(std::string gr_file1, std::string gr_file2, std::vector<Edge> &edges, size_t &graph_size);
bool load_gr_files(std::vector<std::string> gr_files, std::vector<Edge> &edges, size_t &graph_size);
bool load_queries(std::string query_file, std::vector<std::pair<size_t, size_t>> &queries_out);
bool load_queries(std::string query_file, std::vector<std::tuple<size_t, size_t,size_t>> &queries_out);

void split_string(std::string string, std::string delimiter,
                  std::vector<std::string> &results);

void log_sol_cost(std::string filename, SolutionSet solutions);

#endif //UTILS_IO_UTILS_H
