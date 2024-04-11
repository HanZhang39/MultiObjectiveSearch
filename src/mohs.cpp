#include <climits>
#include <cstddef>
#include <iostream>
#include <memory>
#include <time.h>
#include <fstream>

#include "AbstractSolver.h"
#include "ApexSearch.h"
#include "AnytimeApexSearch.h"
#include "WCApexSearch.h"
#include "BOAStar.h"
#include "AnytimeBOA.h"
#include "GCL_bucket.h"
#include "ShortestPathHeuristic.h"
#include "LTMOA.h"
#include "LTMOA2.h"
#include "Utils/Definitions.h"
#include "Utils/IOUtils.h"

#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <boost/filesystem.hpp>

using namespace std;
namespace po = boost::program_options;

const std::string resource_path = "resources/";
// const std::string output_path = "output/";
const std::string output_path = "";
using namespace std::chrono_literals;

std::string dir_loc_sol;
bool log_sol;

void single_run_map(AdjacencyMatrix& graph, AdjacencyMatrix&inv_graph, size_t source, size_t target, std::ofstream& output, std::string algorithm , double eps, unsigned int time_limit, po::variables_map& vm) {
    int num_exp, num_gen;
    long long num_vec_comp;

    std::shared_ptr<AbstractSolver> solver;
    EPS eps_vec(graph.get_num_of_objectives(), eps);

    if (algorithm == "BOA"){
        solver = std::make_unique<BOAStarBucket>(graph, inv_graph, source, target);
    }else if (algorithm == "ABOA"){
        solver = std::make_unique<AnytimeBOA>(graph, inv_graph, source, target, 4);
    }else if (algorithm == "Apex"){
        solver = get_Astarpex_solver(graph, inv_graph, source, target, eps_vec);
    }else if (algorithm == "AnytimeApex"){
        if (vm["param1"].as<double>() > 0){
            decrease_factor = vm["param1"].as<double>();
        }
        solver = get_anytime_Apex_solver(graph, inv_graph, source, target, eps_vec, 2);
    }else if (algorithm == "AnytimeApexRestart"){
        if (vm["param1"].as<double>() > 0){
            decrease_factor = vm["param1"].as<double>();
        }
        solver = get_anytime_Apex_solver(graph, inv_graph, source, target, eps_vec, 3);
    }else if (algorithm == "AnytimeApexEnh"){
        if (vm["param1"].as<double>() > 0){
            decrease_factor =  vm["param1"].as<double>();
        }
        solver = get_anytime_Apex_solver(graph, inv_graph, source, target, eps_vec, 4);
    }else if (algorithm == "AnytimeApexHybrid"){
        if (vm["param1"].as<double>() > 0){
            decrease_factor = vm["param1"].as<double>();
        }
        if (vm["param2"].as<double>() > 0){
            default_hybrid_param =  vm["param2"].as<double>();
        }
        solver = get_anytime_Apex_solver(graph, inv_graph, source, target, eps_vec, 5);
    }else if (algorithm == "WCApex"){
        int bound = vm["bound"].as<int>();
        if (bound < 0){
            std::cerr << "no valid resource bound is given" << endl;
            exit(-1);
        }
        solver = std::make_unique <WCApexSearch>(graph, inv_graph, source, target, bound, eps);
    }else if (algorithm == "LTMOA"){
        solver = get_LTMOA_solver(graph, inv_graph, source, target);
    }else if (algorithm == "LTMOAeps"){
        solver = get_LTMOA_solver(graph, inv_graph, source, target, 0, eps_vec);
    }else if (algorithm == "LTMOAR"){
        use_R2 = true;
        solver = get_LTMOA2_solver(graph, inv_graph, source, target, false);
    }else if (algorithm == "LTMOAR1"){
        solver = get_LTMOA2_solver(graph, inv_graph, source, target, false);
    }else if (algorithm == "LTMOAR2"){
        use_R2 = true;
        solver = get_LTMOA_solver(graph, inv_graph, source, target);
    }else if (algorithm == "LTMOARBucket"){
        use_R2 = true;
        int param1 = 20000;
        if (vm["param1"].as<double>() > 0){
          param1 = (int) vm["param1"].as<double>();
        }
        bucket_step = param1;
        solver = get_LTMOA2_solver(graph, inv_graph, source, target, true);
    }else if (algorithm == "LTMOABucket"){
      int param1 = 20000;
      if (vm["param1"].as<double>() > 0){
        param1 = (int) vm["param1"].as<double>();
      }
        bucket_step = param1;
        solver = get_LTMOA_solver(graph, inv_graph, source, target, 1);
    // }else if (algorithm == "LTMOANDTree"){
    //     int param1 = 5;
    //     int param2 = 20;
    //     if (vm["param1"].as<double>() > 0){
    //         param1 = (int) vm["param1"].as<double>();
    //     }
    //     if (vm["param2"].as<double>() > 0){
    //         param2 = (int) vm["param2"].as<double>();
    //     }
    //     maxBranches = param1;
    //     maxListSize = param2;
    //     solver = get_LTMOA_solver(graph, inv_graph, source, target, 2);
    }else if (algorithm == "LazyLTMOA"){
        solver = get_LTMOA_solver(graph, inv_graph, source, target, -1);
    }else {
        exit(-1);
    }

    solver->verbal = vm["verbal"].as<int>();

    solver->start_time_ = std::chrono::steady_clock::now();
    solver->solve(time_limit);
    auto end = std::chrono::steady_clock::now();
    auto start = solver->start_time_;
    printf("Work took %f seconds\n", (end - start)/1.0s  );
    double runtime = ( end - start )/1.0s;


    std::cout << "Node expansion: " << solver->get_num_expansion() << std::endl;
    std::cout << "Runtime: " << runtime << std::endl;
    num_exp = solver->get_num_expansion();
    num_gen = solver->get_num_generation();
    num_vec_comp = __is_dominating_called_cnt__;

    if (log_sol){
        std::ostringstream stringStream;
        stringStream << dir_loc_sol << "/" << source << "_" << target << ".txt";
        log_sol_cost(stringStream.str(), solver->get_solution_log());
    }

    output << solver->get_result_str() ;
    // output << num_vec_comp << "\t" ;
    output << runtime 
           << std::endl;

    std::cout << "-----End Single Example-----" << std::endl;
}

void run_query(AdjacencyMatrix & graph, AdjacencyMatrix & inv_graph, std::string query_file, std::string output_file, std::string algorithm, double eps, int time_limit, po::variables_map& vm) {
    std::ofstream stats;
    stats.open(output_path + output_file, std::fstream::app);

    std::vector<std::pair<size_t, size_t>> queries;
    if (load_queries(query_file, queries) == false) {
        std::cout << "Failed to load queries file" << std::endl;
        return;
    }

    for (int i = vm["from"].as<int>(); i < std::min((int)queries.size(), vm["to"].as<int>()); i++){

        std::cout << "Started Query: " << i << "/" << std::min((int)queries.size(), vm["to"].as<int>()) << std::endl;
        size_t source = queries[i].first;
        size_t target = queries[i].second;

        single_run_map(graph, inv_graph, source, target, stats, algorithm, eps, time_limit, vm);
    }

}

int main(int argc, char** argv){

    std::vector<string> objective_files;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("start,s", po::value<int>()->default_value(-1), "start location")
        ("goal,g", po::value<int>()->default_value(-1), "goal location")
        ("query,q", po::value<std::string>()->default_value(""), "the query file")
        ("from", po::value<int>()->default_value(0), "start from the i-th line of the query file")
        ("to", po::value<int>()->default_value(INT_MAX), "up to the i-th line of the query file")
        ("map,m",po::value< std::vector<string> >(&objective_files)->multitoken(), "files for edge weight")
        ("algorithm,a", po::value<std::string>()->default_value("Apex"), "solvers (BOA, PPA or Apex search)")
        ("cutoffTime,t", po::value<int>()->default_value(300), "cutoff time (seconds)")
        ("bound", po::value<int>()->default_value(-1), "resource bound")
        ("logsolutions", po::value<std::string>()->default_value(""), "if non-empty, dump solution cost to the directory")
        ("output,o", po::value<std::string>()->required(), "Name of the output file")
        ("eps,e", po::value<double>()->default_value(0), "epsilon values for approximate search")
        ("merge", po::value<std::string>()->default_value(""), "strategy for merging apex node pair: SMALLER_G2, RANDOM or MORE_SLACK")
        ("verbal", po::value<int>()->default_value(0), "level of the output details")
        ("param1", po::value<double>()->default_value(-1), "param 1")
        ("param2", po::value<double>()->default_value(-1), "param 2");


    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    po::notify(vm);
    srand((int)time(0));

    if (vm["query"].as<std::string>() != ""){
        if (vm["start"].as<int>() != -1 || vm["goal"].as<int>() != -1){
            std::cerr << "query file and start/goal cannot be given at the same time !" << std::endl;
            return -1;
        }
    }
    
    dir_loc_sol = vm["logsolutions"].as<std::string>();
    log_sol = dir_loc_sol.size()> 0;
    if (log_sol){
        boost::filesystem::create_directory(dir_loc_sol);
    }

    size_t graph_size;
    std::vector<Edge> edges;

    for (auto file:objective_files){
        std::cout << file << std::endl;
    }

    if (load_gr_files(objective_files, edges, graph_size) == false) {
        std::cout << "Failed to load gr files" << std::endl;
        return -1;
    }

    std::cout << "Graph Size: " << graph_size << std::endl;

    // Build graphs
    AdjacencyMatrix graph(graph_size, edges);
    AdjacencyMatrix inv_graph(graph_size, edges, true);


    if (vm["query"].as<std::string>() != ""){
        run_query(graph, inv_graph, vm["query"].as<std::string>(), vm["output"].as<std::string>(), vm["algorithm"].as<std::string>(), vm["eps"].as<double>(), vm["cutoffTime"].as<int>(), vm);
    } else{
        std::ofstream stats;
        stats.open(vm["output"].as<std::string>(), std::fstream::app);

        single_run_map(graph, inv_graph, vm["start"].as<int>(), vm["goal"].as<int>(), stats, vm["algorithm"].as<std::string>(), vm["eps"].as<double>(), vm["cutoffTime"].as<int>(), vm);
    }

    return 0;
}
