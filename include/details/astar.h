#include "../astar.h"

#include <unordered_map>
#include <vector>

#include <boost/functional/hash.hpp>

#include "data.h"
#include "heuristic.h"
#include "node.h"
#include "trie.h"

using std::vector;
using std::unordered_map;
using planning::TrieTable;
using planning::ptr_t;

namespace planning {

template<class T>
int Expand(ptr_t node, T &heuristic, const vector<int> &fact_offset,
           const vector<var_value_t> &goal, const Actions &actions,
           const TrieTable &table, vector< vector<ptr_t> > &open_list,
           unordered_map<size_t, int> &closed_list, int min_f) {
  ++expanded;
  auto result = table.Find(node->variables, fact_offset);
  for (auto i : result) {
    std::vector<int> new_variables = node->variables;
    ApplyEffect(actions.effects[i], new_variables);
    size_t hash = boost::hash_range(new_variables.begin(),
                                    new_variables.end());
    int g = node->g + actions.costs[i];
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
    int h = heuristic(child->variables, fact_offset, goal, actions);
    ++evaluated;
    if (h == -1) continue;
    int f = child->g + h;
    if (open_list.size() <= f) {
      open_list.resize(f+1);
      open_list[f].reserve(10000);
    }
    open_list[f].push_back(child);
    min_f = f < min_f ? f : min_f;
  }
  return min_f;
}

template<class T>
ptr_t AstarSearch(const vector<int> &initial, const vector<int> &fact_offset,
                  const vector<var_value_t> &goal, Actions &actions,
                  const TrieTable &table) {
  vector< vector<ptr_t> > open_list;
  unordered_map<size_t, int> closed_list;
  closed_list.reserve(100000);
  T heuristic;
  heuristic.Initialize(fact_offset, goal, actions);

  ++generated;
  auto root = Node::Construct();
  root->variables = initial;
  int min_h = heuristic(initial, fact_offset, goal, actions);
  if (min_h == -1) return nullptr;
  ++evaluated;
  int min_f = min_h;
  open_list.resize(min_f+1);
  for (int i=0, n=open_list.size(); i<n; ++i)
    open_list[i].reserve(10000);
  open_list[min_f].push_back(root);

  int pre_min_h = min_h + 1;
  while (!open_list[min_f].empty()) {
    auto node = open_list[min_f].back();
    min_h = min_f - node->g;
    if (min_h < pre_min_h) {
      PrintNewHeuristicValue(min_h, node->g);
      PrintNewFValue(min_f);
      pre_min_h = min_h;
    }
    if (GoalCheck(goal, node->variables)) return node;
    size_t hash = boost::hash_range(node->variables.begin(),
                                    node->variables.end());
    closed_list[hash] = node->g;
    open_list[min_f].pop_back();
    min_f = Expand<T>(node, heuristic, fact_offset, goal, actions, table,
                      open_list, closed_list, min_f);
    while (open_list[min_f].empty()) {
      if (++min_f >= open_list.size()) return nullptr;
    }
  }
  return nullptr;
}

} // namespace astar
