#include <chrono>
#include <iostream>
#include <map>
#include <queue>
#include <unordered_map>
#include <vector>

#include <boost/functional/hash.hpp>

#include "data.h"
#include "node.h"
#include "parser.h"
#include "trie.h"

using planning::var_value_t;
using planning::Node;

int expanded = 0;
int generated = 0;

void Expand(
    boost::intrusive_ptr<Node> node,
    const std::vector<int> &fact_offset,
    const std::vector<int> &costs,
    const std::vector< std::vector<var_value_t> > &effects,
    const planning::TrieTable &table,
    std::queue< boost::intrusive_ptr<Node> > &open_list,
    std::unordered_map<size_t, int> &closed_list) {
  ++expanded;
  auto result = table.Find(node->variables, fact_offset);
  for (auto i : result) {
    std::vector<int> new_variables = node->variables;
    planning::ApplyEffect(effects[i], new_variables);
    size_t hash = boost::hash_range(new_variables.begin(),
                                    new_variables.end());
    int g = node->g + costs[i];
    if (closed_list.find(hash) != closed_list.end()
        && closed_list[hash] <= g) {
      continue;
    }
    auto child = Node::Construct();
    ++generated;
    child->variables = std::move(new_variables);
    child->action = i;
    child->g = g;
    child->parent_node = node;
    open_list.push(child);
  }
}

boost::intrusive_ptr<Node> BFS(
    const std::vector<int> &variables,
    const std::vector<int> &fact_offset,
    const std::vector<var_value_t> &goal,
    const planning::Actions &actions,
    const planning::TrieTable &table) {
  std::queue< boost::intrusive_ptr<Node> > open_list;
  std::unordered_map<size_t, int> closed_list;
  auto node = Node::Construct();
  node->variables = variables;
  ++generated;
  open_list.push(node);
  while (!open_list.empty()) {
    node = open_list.front();
    open_list.pop();
    if (planning::GoalCheck(goal, node->variables)) return node;
    size_t hash = boost::hash_range(node->variables.begin(),
                                    node->variables.end());
    closed_list[hash] = node->g;
    Expand(node, fact_offset, actions.costs, actions.effects, table,
           open_list, closed_list);
  }
  return nullptr;
}

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: bfs <filename>" << std::endl;
    exit(1);
  }
  std::string filename = argv[1];
  std::vector<int> initial;
  std::vector<int> fact_offset;
  std::vector< std::vector<var_value_t> > mutex_groups;
  std::vector<var_value_t> goal;
  planning::Actions actions;
  planning::Parse(filename, initial, fact_offset, mutex_groups, goal,
                  &actions);
  auto chrono_start = std::chrono::system_clock::now();
  auto table = planning::TrieTable::Construct(actions.preconditions,
                                              fact_offset);
  auto result = BFS(initial, fact_offset, goal, actions, table);
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
    std::cout << name_array.back() << " (" << cost << ")" << std::endl;
    cost_array.pop_back();
    name_array.pop_back();
  }
  std::cout << "Plan length: " << step << " step(s)" << std::endl;
  std::cout << "Plan cost: " << result_cost << std::endl;
  std::cout << "Expanded " << expanded << " state(s)" << std::endl;
  std::cout << "Generated " << generated << " state(s)" << std::endl;
  std::cout << "Search time: " << search_time << "s" << std::endl;
  std::cout << "Solution found." << std::endl;
}
