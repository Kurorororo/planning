#ifndef GOAL_COUNT_H_
#define GOAL_COUNT_H_

#include <memory>
#include <unordered_map>
#include <vector>

#include "heuristic.h"

namespace heuristic {

class GoalCount : public HeuristicInterface<GoalCount> {
 public:
  int operator()(const std::vector<int> &variables,
                 const std::unordered_map<int, int> &goal);
};

} // namespace goal_count

#endif
