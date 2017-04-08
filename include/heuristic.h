#ifndef HEURISTIC_H_
#define HEURISTIC_H_

#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace heuristic {

template<class T>
class HeuristicInterface {
 public:
  int operator()(const std::vector<int> &variables,
                 const std::unordered_map<int, int> &goal) {
    return static_cast<T &>(this)->operator()(variables, goal);
  }

};

template<class T>
class Heuristic {
  static_assert(std::is_base_of<HeuristicInterface<T>, T>::value,
                "T is not extended interface class");
 public:
  int operator()(const std::vector<int> &variables,
                 const std::unordered_map<int, int> &goal) {
    return object_(variables, goal);
  }

  T object_;
};

}

#endif // HEURISTIC_H_
