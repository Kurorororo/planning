#ifndef ASTAR_H_
#define ASTAR_H_

#include <iostream>
#include <type_traits>
#include <vector>

#include "data.h"
#include "node.h"
#include "trie.h"

namespace planning {

int evaluated = 0;
int expanded = 0;
int generated = 0;

template<class T>
ptr_t AstarSearch(const std::vector<int> &initial,
                  const std::vector<int> &fact_offset,
                  const std::vector<var_value_t> &goal,
                  const Actions &actions, const TrieTable &table);

inline void PrintNewHeuristicValue(int min_h, int g) {
  std::cout << "New best heuristic value: " << min_h << std::endl;
  std::cout << "[g=" << g << ", " << evaluated << " evaluated, "
            << expanded << " expanded]" << std::endl;
}

inline void PrintNewFValue(int min_f) {
  std::cout << "f = " << min_f << " [" << evaluated << " evaluated, "
            << expanded << " expanded]" << std::endl;
}

} // namespace planning

#include "details/astar.h"

#endif // ASTAR_H_
