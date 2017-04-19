#ifndef MRW_H_
#define MRW_H_

#include <iostream>
#include <random>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "data.h"
#include "node.h"
#include "trie.h"

namespace planning {

int generated;
int expanded;
int evaluated;

template<class T>
ptr_t MRW(const std::vector<int> &initial, const std::vector<int> &fact_offset,
          const std::vector<var_value_t> &goal, const Actions &actions,
          const TrieTable &table, double alpha=0.9, int num_walk=2000,
          int length_walk=10, int extending_period=300,
          double extending_rate=1.5, int max_steps=7);

inline void PrintNewHeuristicValue(int min_h, int g) {
  std::cout << "New best heuristic value: " << min_h << std::endl;
  std::cout << "[g=" << g << ", " << evaluated << " evaluated, "
            << expanded << " expanded]" << std::endl;
}

} // namespace planning

#include "details/mrw.h"

#endif // MRW_H_
