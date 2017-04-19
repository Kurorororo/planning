#ifndef HEURISTIC_H_
#define HEURISTIC_H_

#include <type_traits>
#include <vector>

#include "data.h"

namespace planning {

template<class T>
class HeuristicInterface {
 public:
  void Initialize(const std::vector<int> &fact_offset,
                  const std::vector<var_value_t> &goal,
                  const Actions &actions) {
    static_cast<T &>(this)->Initialize(fact_offset, goal, actions);
  }

  int operator()(const std::vector<int> &variables,
                 const std::vector<int> &fact_offset,
                 const std::vector<var_value_t> &goal,
                 const Actions &actions) {
    return static_cast<T &>(this)->operator()(variables, fact_offset, goal,
                                              actions);
  }

};

template<class T>
class Heuristic {
  static_assert(std::is_base_of<HeuristicInterface<T>, T>::value,
                "T is not extended interface class");
 public:
  void Initialize(const std::vector<int> &fact_offset,
                  const std::vector<var_value_t> &goal,
                  const Actions &actions) {
    object_.Initialize(fact_offset, goal, actions);
  }

  int operator()(const std::vector<int> &variables,
                 const std::vector<int> &fact_offset,
                 const std::vector<var_value_t> &goal,
                 const Actions &actions) {
    return object_(variables, fact_offset, goal, actions);
  }

  T object_;
};

} // planning

#endif // HEURISTIC_H_
