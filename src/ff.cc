#include "goalcount.h"

#include <unordered_map>
#include <vector>

#include "heuristic.h"

namespace heuristic {

int FF::operator()(const std::vector<int> &variables,
                   const std::unordered_map<int, int> &goal) {
  int count = goal.size();
  for (auto v : goal)
    if (variables[v.first] == v.second) --count;
  return count;
}

} // namespace heuristic
