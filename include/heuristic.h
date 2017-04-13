#ifndef HEURISTIC_H_
#define HEURISTIC_H_

#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "sas_data.h"

namespace heuristic {

template<class T>
class HeuristicInterface {
 public:   
  void Initialize(
      const std::vector<int> &sups,
      const std::unordered_map<int, int> &goal,
      const std::vector< std::map<int, int> > &preconditions,
      const std::vector< std::unique_ptr< sas_data::SASOperator> >
          &sas_operators) {
    static_cast<T &>(this)->Initialize(sups, goal, preconditions,
                                       sas_operators);
  }

  int operator()(
    const std::vector<int> &variables,
    const std::unordered_map<int, int> &goal,
    const std::vector< std::map<int, int> > &preconditions,
    const std::vector< std::unique_ptr< sas_data::SASOperator> >
        &sas_operators) {
    return static_cast<T &>(this)->operator()(variables, goal, preconditions,
                                              sas_operators);
  }

};

template<class T>
class Heuristic {
  static_assert(std::is_base_of<HeuristicInterface<T>, T>::value,
                "T is not extended interface class");
 public:
  void Initialize(
      const std::vector<int> &sups,
      const std::unordered_map<int, int> &goal,
      const std::vector< std::map<int, int> > &preconditions,
      const std::vector< std::unique_ptr< sas_data::SASOperator> >
          &sas_operators) {
    object_.Initialize(sups, goal, preconditions, sas_operators);
  }

  int operator()(
    const std::vector<int> &variables,
    const std::unordered_map<int, int> &goal,
    const std::vector< std::map<int, int> > &preconditions,
    const std::vector< std::unique_ptr< sas_data::SASOperator> >
        &sas_operators) {
    return object_(variables, goal, preconditions, sas_operators);
  }

  T object_;
};

}

#endif // HEURISTIC_H_
