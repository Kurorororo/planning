#include "../mrw.h"

#include <algorithm>
#include <random>
#include <vector>

#include "data.h"
#include "node.h"
#include "trie.h"

using std::vector;

namespace planning {

template<class T>
ptr_t PureRandomWalk(T &heuristic, int h_min_old,
                     const vector<int> &fact_offset,
                     const vector<var_value_t> &goal, const Actions &actions,
                     const TrieTable &table, ptr_t s, double alpha,
                     int num_walk, int length_walk, int extending_period,
                     double extending_rate, int max_step) {
  int h_min = -1;
  ptr_t s_min = nullptr;
  int counter = 0;
  std::cout << "New length of random walk: " << length_walk << std::endl;
  double p, ap;
  std::random_device seed_gen;
  std::default_random_engine engine(seed_gen());
  for (int i=0; i<num_walk; ++i) {
    ptr_t s_prime = s;
    for (int j=0; j<length_walk; ++j) {
      auto a_set = table.Find(s_prime->variables, fact_offset);
      if (a_set.empty()) break;
      std::uniform_int_distribution<> dist(0, a_set.size()-1);
      int a = a_set[dist(engine)];
      ++expanded;
      ++generated;
      auto child = Node::Construct();
      child->variables = s_prime->variables;
      ApplyEffect(actions.effects[a], child->variables);
      child->action = a;
      child->parent_node = s_prime;
      child->g = s_prime->g + actions.costs[a];
      if (GoalCheck(goal, child->variables)) return child;
      s_prime = child;
    }
    int h = heuristic(s_prime->variables, fact_offset, goal, actions);
    ++evaluated;
    if ((h < h_min || h_min == -1) && h != -1) {
      s_min = s_prime;
      h_min = h;
      counter = 0;
    } else {
      ++counter;
      if (counter > extending_period) {
        length_walk = static_cast<int>(
            extending_rate * static_cast<double>(length_walk));
        counter = 0;
        std::cout << "New length of random walk: " << length_walk << std::endl;
      }
    }
    p = std::max(0.0, static_cast<double>(h_min_old - h_min));
    if (i == 0)
      ap = p;
    if (p > ap) {
      std::cout << "Exploration stopped " << i+1 << " random walks"
                << std::endl;
      return s_min;
    }
    ap = (1.0-alpha)*ap + alpha*p;
  }
  std::cout << "Exploration stopped " << num_walk << " random walks"
            << std::endl;
  if (s_min == nullptr)
    return s;
  else
    return s_min;
}

template<class T>
ptr_t MRW(const vector<int> &initial, const vector<int> &fact_offset,
          const vector<var_value_t> &goal, const Actions &actions,
          const TrieTable &table, double alpha, int num_walk, int length_walk,
          int extending_period, double extending_rate, int max_steps) {
  T heuristic;
  heuristic.Initialize(fact_offset, goal, actions);

  ++generated;
  ptr_t s_zero = Node::Construct();
  s_zero->variables = initial;
  ptr_t s = s_zero;
  int initial_h_min = heuristic(initial, fact_offset, goal, actions);
  if (initial_h_min == -1) return nullptr;
  int h_min = initial_h_min;
  ++evaluated;
  int counter = 0;
  while (!GoalCheck(goal, s->variables)) {
    if (counter > max_steps|| table.Find(s->variables, fact_offset).empty()) {
      s = s_zero;
      counter = 0;
      h_min = initial_h_min;
      std::cout << "Restart" << std::endl;
      PrintNewHeuristicValue(h_min, s->g);
    }
    s = PureRandomWalk<T>(heuristic, h_min, fact_offset, goal, actions,
                          table, s, alpha, num_walk, length_walk,
                          extending_period, extending_rate, max_steps);
    int h = heuristic(s->variables, fact_offset, goal, actions);
    if (h < h_min) {
      h_min = h;
      counter = 0;
      PrintNewHeuristicValue(h_min, s->g);
    } else {
      ++counter;
    }
  }
  return s;
}

} // namespace planning
