#ifndef MRW_H_
#define MRW_H_

#include <iostream>
#include <random>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "heuristic.h"
#include "node.h"
#include "sas_data.h"
#include "trie.h"

namespace mrw {

template<class T>
class MRW {
  static_assert(std::is_base_of<heuristic::HeuristicInterface<T>, T>::value,
                "T is not extended interface class");
 public:
  MRW()
      : evaluated_(0),
        expanded_(0),
        generated_(0),
        num_walk_(2000),
        length_walk_(10) {
          std::random_device seed_gen;
          std::default_random_engine engine(seed_gen());
          engine_ = engine;
        }

  ~MRW() {}

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

  void PrintNewHeuristicValue(int min_h, int g) {
    std::cout << "New best heuristic value: " << min_h << std::endl;
    std::cout << "[g=" << g << ", "
             << evaluated_ << " evaluated, "
             << expanded_ << " expanded]" << std::endl;
  }

 private:
  node::ptr_t PureRandomWalk(
      const std::unordered_map<int, int> &goal,
      const std::vector< std::unique_ptr<sas_data::SASOperator> >
          &sas_operators,
      const trie::TrieTable &table,
      node::ptr_t s);

  T heuristic;
  std::default_random_engine engine_;
  int max_step_;
  int num_walk_;
  int length_walk_;
  int evaluated_;
  int expanded_;
  int generated_;
};

} // namespace mrw

#include "details/mrw.h"

#endif // MRW_H_
