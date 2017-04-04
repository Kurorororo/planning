#include "goal_count.h"

#include <memory>
#include <unordered_map>
#include <vector>

#include "heuristic.h"

namespace heuristic {

int GoalCount::operator()(const std::vector<int> &variables,
                          const std::unordered_map<int, int> &goal) {
  int count = variables.size();
  for (auto v : goal)
    if (variables[v.first] == v.second) --count;
  return count;
}

} // namespace heuristic
