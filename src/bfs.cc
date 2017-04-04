#include <chrono>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <vector>

#include <boost/functional/hash.hpp>

#include "node.h"
#include "parser.h"
#include "sas_data.h"
#include "trie.h"

int expanded = 0;
int generated = 0;

void Expand(
    boost::intrusive_ptr<node::Node> node,
    const std::vector< std::unique_ptr<sas_data::SASOperator> > &sas_operators,
    const trie::TrieTable &table,
    std::queue< boost::intrusive_ptr<node::Node> > &open_list,
    std::unordered_map<size_t, int> &closed_list) {
  ++expanded;
  std::vector<int> result;
  table.Find(node->variables_, result);
  for (auto i : result) {
    std::vector<int> new_variables = node->variables_;
    sas_operators[i]->ApplyEffects(new_variables);
    size_t hash = boost::hash_range(new_variables.begin(),
                                    new_variables.end());
    int cost = node->get_cost()+sas_operators[i]->get_cost();
    if (closed_list.find(hash) != closed_list.end()
        && closed_list[hash] <= cost) {
      continue;
    }
    auto child = node::Node::Construct();
    ++generated;
    child->variables_ = std::move(new_variables);
    child->set_action(sas_operators[i]->get_name());
    child->set_cost(cost);
    child->set_step(node->get_step()+1);
    child->set_parent_node(node);
    open_list.push(child);
  }
}

boost::intrusive_ptr<node::Node> BFS(
    const std::vector<int> &variables,
    const std::unordered_map<int, int> &goal,
    const std::vector< std::unique_ptr<sas_data::SASOperator> > &sas_operators,
    const trie::TrieTable &table) {
  std::queue< boost::intrusive_ptr<node::Node> > open_list;
  std::unordered_map<size_t, int> closed_list;
  auto node = node::Node::Construct();
  node->variables_ = variables;
  ++generated;
  open_list.push(node);
  while (!open_list.empty()) {
    node = open_list.front();
    open_list.pop();
    if (sas_data::GoalCheck(node->variables_, goal)) return node;
    size_t hash = boost::hash_range(node->variables_.begin(),
                                    node->variables_.end());
    closed_list[hash] = node->get_cost();
    Expand(node, sas_operators, table, open_list, closed_list);
  }
  return nullptr;
}

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: bfs <filename>" << std::endl;
    exit(1);
  }
  std::string filename = argv[1];
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
  auto result = BFS(variables, goal, sas_operators, table);
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
  std::cout << "Expanded " << expanded << " state(s)" << std::endl;
  std::cout << "Generated " << generated << " state(s)" << std::endl;
  std::cout << "Search time: " << search_time << "s" << std::endl;
  std::cout << "Solution found." << std::endl;
}
