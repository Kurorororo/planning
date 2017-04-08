#include "../astar.h"

#include <unordered_map>
#include <vector>

#include <boost/functional/hash.hpp>

#include "heuristic.h"
#include "node.h"
#include "sas_data.h"
#include "trie.h"

using std::unique_ptr;
using std::unordered_map;
using std::vector;
using sas_data::SASOperator;
using trie::TrieTable;
using node::ptr_t;

namespace astar {

template<class T>
ptr_t AstarSearch<T>::Search(
    const vector<int> &variables,
    const unordered_map<int, int> &goal,
    const vector< unique_ptr<SASOperator> > &sas_operators,
    const TrieTable &table) {
  vector< vector<ptr_t> > open_list;
  unordered_map<size_t, int> closed_list;
  closed_list.reserve(100000);
  T heuristic;

  ++generated_;
  auto node = node::Node::Construct();
  node->variables_ = variables;
  int min_h = heuristic(variables, goal);
  ++evaluated_;
  int min_f = min_h;
  open_list.resize(min_f+1);
  for (int i=0, n=open_list.size(); i<n; ++i)
    open_list[i].reserve(10000);
  open_list[min_f].push_back(node);

  int pre_min_h = min_h + 1;
  int pre_min_f = -1;
  while (!open_list[min_f].empty()) {
    node = open_list[min_f].back();
    open_list[min_f].pop_back();

    min_h = min_f - node->get_g();
    if (min_h < pre_min_h) {
      PrintNewHeuristicValue(min_h, node->get_g());
      pre_min_h = min_h;
    }
    if (min_f != pre_min_f) {
      PrintNewFValue(min_f);
      pre_min_f = min_f;
    }

    if (sas_data::GoalCheck(node->variables_, goal)) return node;
    size_t hash = boost::hash_range(node->variables_.begin(),
                                    node->variables_.end());
    closed_list[hash] = node->get_g();
    min_f = Expand(heuristic, node, goal, sas_operators, table, open_list,
                   closed_list, min_f);
    while (open_list[min_f].empty()) {
      if (++min_f >= open_list.size()) return nullptr;
    }
  }
  return nullptr;
}

template<class T>
int AstarSearch<T>::Expand(
    T heuristic,
    ptr_t node,
    const unordered_map<int, int> &goal,
    const vector< unique_ptr<SASOperator> > &sas_operators,
    const TrieTable &table,
    vector< vector<ptr_t> > &open_list,
    unordered_map<size_t, int> &closed_list,
    int min_f) {
  ++expanded_;
  auto result = table.Find(node->variables_);
  for (auto i : result) {
    std::vector<int> new_variables = node->variables_;
    sas_operators[i]->ApplyEffects(new_variables);
    size_t hash = boost::hash_range(new_variables.begin(),
                                    new_variables.end());
    int g = node->get_g() + sas_operators[i]->get_cost();
    if (closed_list.find(hash) != closed_list.end()
        && closed_list[hash] <= g) {
      continue;
    }
    auto child = node::Node::Construct();
    ++generated_;
    child->variables_ = std::move(new_variables);
    child->set_action(sas_operators[i]->get_name());
    child->set_g(g);
    child->set_step(node->get_step()+1);
    child->set_parent_node(node);
    int f = child->get_g() + heuristic(child->variables_, goal);
    ++evaluated_;
    if (open_list.size() <= f) open_list.resize(f+1);
    open_list[f].reserve(10000);
    open_list[f].push_back(child);
    min_f = f < min_f ? f : min_f;
  }
  return min_f;
}

} // namespace astar
