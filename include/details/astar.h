#include "../astar.h"

#include <unordered_map>
#include <vector>

#include <boost/functional/hash.hpp>

#include "heuristic.h"
#include "heuristic_node.h"
#include "sas_data.h"
#include "trie.h"

using std::unique_ptr;
using std::unordered_map;
using std::vector;
using sas_data::SASOperator;
using trie::TrieTable;
using heuristic_node::ptr_t;

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
  T functor;

  ++generated_;
  auto node = heuristic_node::HeuristicNode::Construct();
  node->variables_ = variables;
  node->set_heuristic(functor(variables, goal));

    int min = node->get_heuristic();
    open_list.resize(min+1);
    for (int i=0, n=open_list.size(); i<n; ++i)
      open_list[i].reserve(10000);
    open_list[min].push_back(node);

    while (!open_list[min].empty()) {
      node = open_list[min].back();
      open_list[min].pop_back();
      if (sas_data::GoalCheck(node->variables_, goal)) return node;
      size_t hash = boost::hash_range(node->variables_.begin(),
                                      node->variables_.end());
      closed_list[hash] = node->get_cost();
      min = Expand(functor, node, goal, sas_operators, table, open_list,
                   closed_list, min);
      while (open_list[min].empty()) {
        if (++min > open_list.size()) return nullptr;
      }
    }
    return nullptr;
  }

template<class T>
int AstarSearch<T>::Expand(
    T functor,
    ptr_t node,
    const unordered_map<int, int> &goal,
    const vector< unique_ptr<SASOperator> > &sas_operators,
    const TrieTable &table,
    vector< vector<ptr_t> > &open_list,
    unordered_map<size_t, int> &closed_list,
    int min) {
  ++expanded_;
  auto result = table.Find(node->variables_);
  for (auto i : result) {
    std::vector<int> new_variables = node->variables_;
    sas_operators[i]->ApplyEffects(new_variables);
    size_t hash = boost::hash_range(new_variables.begin(),
                                    new_variables.end());
    int cost = node->get_cost() + sas_operators[i]->get_cost();
    if (closed_list.find(hash) != closed_list.end()
        && closed_list[hash] <= cost) {
      continue;
    }

    auto child = heuristic_node::HeuristicNode::Construct();
    ++generated_;
    child->variables_ = std::move(new_variables);
    child->set_action(sas_operators[i]->get_name());
    child->set_cost(cost);
    child->set_heuristic(functor(child->variables_, goal));
    child->set_step(node->get_step()+1);
    child->set_parent_node(node);

    int f = child->get_cost() + child->get_heuristic();
    if (open_list.size() <= f) open_list.resize(f+1);
    open_list[f].reserve(10000);
    open_list[f].push_back(child);
    min = f < min ? f : min;
  }
  return min;
}

} // namespace astar
