#include "../mrw.h"

#include <algorithm>
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
  int initial_h_min = heuristic(variables, goal);
  h_min_ = initial_h_min;
  ++evaluated_;
  int counter = 0;
  while (!sas_data::GoalCheck(s->variables_, goal)) {
    if (counter > max_steps_ || table.Find(s->variables_).empty()) {
      s = s_zero;
      counter = 0;
      h_min_ = initial_h_min;
      std::cout << "Restart" << std::endl;
      PrintNewHeuristicValue(h_min_, s->get_g());
    }
    s = PureRandomWalk(goal, sas_operators, table, s);
    int h = heuristic(s->variables_, goal);
    if (h < h_min_) {
      h_min_ = h;
      counter = 0;
      PrintNewHeuristicValue(h_min_, s->get_g());
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
  int length_walk = length_walk_;
  int counter = 0;
  std::cout << "New length of random walk: " << length_walk << std::endl;
  double p, ap;
  std::random_device seed_gen;
  std::default_random_engine engine(seed_gen());
  for (int i=0; i<num_walk_; ++i) {
    ptr_t s_prime = s;
    for (int j=0; j<length_walk; ++j) {
      auto a_set = table.Find(s_prime->variables_);
      if (a_set.empty()) break;
      ++expanded_;
      std::uniform_int_distribution<> dist(0, a_set.size()-1);
      int a = a_set[dist(engine)];
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
      counter = 0;
    } else {
      ++counter;
      if (counter > extending_period_) {
        length_walk = static_cast<int>(
            extending_rate_ * static_cast<double>(length_walk));
        counter = 0;
        std::cout << "New length of random walk: " << length_walk << std::endl;
      }
    }
    p = std::max(0.0, static_cast<double>(h_min_ - h_min));
    if (i == 0)
      ap = p;
    if (p > ap) {
      std::cout << "Exploration stopped " << i+1 << " random walks"
                << std::endl;
      return s_min;
    }
    ap = (1.0-alpha_)*ap + alpha_*p;
  }
  std::cout << "Exploration stopped " << num_walk_ << " random walks"
            << std::endl;
  if (s_min == nullptr)
    return s;
  else
    return s_min;
}

} // namespace mrw
