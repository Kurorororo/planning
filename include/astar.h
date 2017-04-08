#ifndef ASTAR_H_
#define ASTAR_H_

#include <iostream>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "heuristic.h"
#include "node.h"
#include "sas_data.h"
#include "trie.h"

namespace astar {

template<class T>
class AstarSearch {
  static_assert(std::is_base_of<heuristic::HeuristicInterface<T>, T>::value,
                "T is not extended interface class");

 public:
  AstarSearch() : generated_(0), expanded_(0) {}
  ~AstarSearch() {}

  node::ptr_t Search(
      const std::vector<int> &variables,
      const std::unordered_map<int, int> &goal,
      const std::vector< std::unique_ptr<sas_data::SASOperator> >
          &sas_operators,
      const trie::TrieTable &table);

  int get_evaluated() const {
    return evaluated_;
  }

  int get_expanded() const {
    return expanded_;
  }

  int get_generated() const {
    return generated_;
  }

 private:
  int Expand(
      T heuristic,
      node::ptr_t node,
      const std::unordered_map<int, int> &goal,
      const std::vector< std::unique_ptr<sas_data::SASOperator> >
          &sas_operators,
      const trie::TrieTable &table,
      std::vector< std::vector<node::ptr_t> > &open_list,
      std::unordered_map<size_t, int> &closed_list,
      int min);

  void PrintNewHeuristicValue(int min_h, int g) {
    std::cout << "New best heuristic value: " << min_h << std::endl;
    std::cout << "[g=" << g << ", "
             << evaluated_ << " evaluated, "
             << expanded_ << " expanded]" << std::endl;
  }

  void PrintNewFValue(int min_f) {
    std::cout << "f = " << min_f << " ["
              << evaluated_ << " evaluated, "
              << expanded_ << " expanded]" << std::endl;
  }

  int evaluated_;
  int expanded_;
  int generated_;
};

} // namespace astar

#include "details/astar.h"

#endif // ASTAR_H_
