#include "../mrw.h"

#include <random>
#include <unordered_map>
#include <vector>

#include "heuristic.h"
#include "node.h"
#include "sas_data.h"
#include "trie.h"

using std::unordered_map;
using std::vector;
using std::unique_ptr;
using sas_data::SASOperator;
using trie::TrieTable;
using node::ptr_t;

namespace mrw {

template<class T>
ptr_t MRW<T>::Search(const vector<int> &variables,
                     const unordered_map<int, int> &goal,
                     const vector< unique_ptr<SASOperator> > &sas_operators,
                     const TrieTable &table) {
  ptr_t s_zero = node::Node::Construct();
  s_zero->variables_ = variables;
  ptr_t s = s_zero;
  ++generated_;
  int h_min = heuristic(variables, goal);
  ++evaluated_;
  int counter = 0;
  while (!sas_data::GoalCheck(s->variables_, goal)) {
    if (counter > max_step_ || table.Find(s->variables_).empty()) {
      s = s_zero;
      counter = 0;
      std::cout << "restart" << std::endl;
    }
    s = PureRandomWalk(goal, sas_operators, table, s);
    int h = heuristic(s->variables_, goal);
    if (h < h_min) {
      h_min = h;
      counter = 0;
    } else {
      ++counter;
    }
  }
  return s;
}

template<class T>
ptr_t MRW<T>::PureRandomWalk(
    const unordered_map<int, int> &goal,
    const vector< unique_ptr<SASOperator> > &sas_operators,
    const TrieTable &table,
    ptr_t s) {
  int h_min = -1;
  ptr_t s_min = nullptr;
  for (int i=0; i<num_walk_; ++i) {
    ptr_t s_prime = s;
    for (int j=0; j<length_walk_; ++j) {
      auto a_set = table.Find(s_prime->variables_);
      if (a_set.empty()) break;
      ++expanded_;
      std::uniform_int_distribution<> dist(0, a_set.size()-1);
      int a = a_set[dist(engine_)];
      auto child = node::Node::Construct();
      ++generated_;
      child->variables_ = s->variables_;
      sas_operators[a]->ApplyEffects(child->variables_);
      child->set_parent_node(s_prime);
      child->set_step(s_prime->get_step()+1);
      child->set_action(sas_operators[a]->get_name());
      child->set_g(s_prime->get_g()+sas_operators[a]->get_cost());
      if (sas_data::GoalCheck(child->variables_, goal)) return child;
      s_prime = child;
    }
    int h = heuristic(s_prime->variables_, goal);
    ++evaluated_;
    if (h_min == -1 || h < h_min) {
      s_min = s_prime;
      h_min = h;
      PrintNewHeuristicValue(h_min, s_min->get_g());
    }
  }
  if (s_min == nullptr)
    return s;
  else
    return s_min;
}

} // namespace mrw
