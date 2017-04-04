#ifndef ASTAR_H_
#define ASTAR_H_

#include <type_traits>
#include <unordered_map>
#include <vector>

#include "heuristic.h"
#include "heuristic_node.h"
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

  heuristic_node::ptr_t Search(
      const std::vector<int> &variables,
      const std::unordered_map<int, int> &goal,
      const std::vector< std::unique_ptr<sas_data::SASOperator> >
          &sas_operators,
      const trie::TrieTable &table);

  int get_expanded() const {
    return expanded_;
  }

  int get_generated() const {
    return generated_;
  }

 private:
  int Expand(
      T functor,
      heuristic_node::ptr_t node,
      const std::unordered_map<int, int> &goal,
      const std::vector< std::unique_ptr<sas_data::SASOperator> >
          &sas_operators,
      const trie::TrieTable &table,
      std::vector< std::vector<heuristic_node::ptr_t> > &open_list,
      std::unordered_map<size_t, int> &closed_list,
      int min);

  int expanded_;
  int generated_;
};

} // namespace astar

#include "details/astar.h"

#endif
