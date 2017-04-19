#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#include "astar.h"
#include "data.h"
#include "parser.h"
#include "trie.h"

namespace planning {

template<class T>
void Run(std::string filename) {
  std::vector<int> initial;
  std::vector<int> fact_offset;
  std::vector< std::vector<var_value_t> > mutex_groups;
  std::vector<var_value_t> goal;
  Actions actions;

  Parse(filename, initial, fact_offset, mutex_groups, goal, &actions);

  auto chrono_start = std::chrono::system_clock::now();
  auto table = TrieTable::Construct(actions.preconditions, fact_offset);
  auto result = AstarSearch<T>(initial, fact_offset, goal, actions, table);
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

  int result_cost = result->g;

  std::vector<std::string> name_array;
  std::vector<int> cost_array;
  int step = 0;
  while (result->parent_node != nullptr) {
    name_array.push_back(actions.names[result->action]);
    cost_array.push_back(result->g);
    ++step;
    result = result->parent_node;
  }

  int sum = 0;
  while (!name_array.empty()) {
    int cost = cost_array.back() - sum;
    sum += cost;
    if (name_array.back() != "")
      std::cout << name_array.back() << " (" << cost << ")" << std::endl;
    cost_array.pop_back();
    name_array.pop_back();
  }

  std::cout << "Plan length: " << step << " step(s)" << std::endl;
  std::cout << "Plan cost: " << result_cost << std::endl;
  std::cout << "Expanded " << expanded << " state(s)" << std::endl;
  std::cout << "Evaluated " << evaluated << " state(s)" << std::endl;
  std::cout << "Generated " << generated << " state(s)" << std::endl;
  std::cout << "Search time: " << search_time << "s" << std::endl;
  std::cout << "Solution found." << std::endl;
}

} // namespace astar
