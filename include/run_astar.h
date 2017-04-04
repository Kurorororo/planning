#include <chrono>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "astar.h"
#include "parser.h"
#include "trie.h"

namespace astar {

template<class T>
void Run(std::string filename) {
  std::vector<int> variables;
  std::vector<int> sups;
  std::vector< std::unordered_map<int, int> > mutex_groups;
  std::unordered_map<int, int> goal;
  std::vector< std::vector<int> > preconditions;
  std::vector< std::unique_ptr<sas_data::SASOperator> > sas_operators;

  parser::Parse(filename, variables, sups, mutex_groups, goal, preconditions,
                sas_operators);

  auto chrono_start = std::chrono::system_clock::now();
  auto table = trie::TrieTable::Construct(preconditions, sups);
  AstarSearch<T> astar;
  auto result = astar.Search(variables, goal, sas_operators, table);
  auto chrono_end = std::chrono::system_clock::now();

  if (result == nullptr) {
    std::cout << "faild to solve problem" << std::endl;
    exit(0);
  }
  std::cout << "Solution found!" << std::endl;

  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
     chrono_end - chrono_start).count();
  double search_time = static_cast<double>(ns) / 1e9;
  std::cout << "Acutual search time: " << search_time << "s" << std::endl;

  int step = result->get_step();
  int cost = result->get_cost();

  std::vector<std::string> actions;
  std::vector<int> costs;
  while (result != nullptr) {
    actions.push_back(result->get_action());
    costs.push_back(result->get_cost());
    result = result->get_parent_node();
  }

  int sum = 0;
  while (!actions.empty()) {
    int cost = costs.back() - sum;
    sum += cost;
    if (actions.back() != "")
      std::cout << actions.back() << " (" << cost << ")" << std::endl;
    costs.pop_back();
    actions.pop_back();
  }

  std::cout << "Plan length: " << step << " step(s)" << std::endl;
  std::cout << "Plan cost: " << step << std::endl;
  std::cout << "Expanded " << astar.get_expanded() << " state(s)";
  std::cout << std::endl;
  std::cout << "Generated " << astar.get_generated() << " state(s)";
  std::cout << std::endl;
  std::cout << "Search time: " << search_time << "s" << std::endl;
  std::cout << "Solution found." << std::endl;
}

} // namespace astar
