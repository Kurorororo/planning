#include "goalcount.h"

#include <vector>

#include "data.h"
#include "heuristic.h"

namespace planning {

int Goalcount::operator()(const std::vector<int> &variables,
                          const std::vector<int> &fact_offset,
                          const std::vector<var_value_t> &goal,
                          const Actions &actions) {
  int count = goal.size();
  for (auto v : goal) {
    int var, value;
    DecodeVarValue(v, &var, &value);
    if (variables[var] == value) --count;
  }
  return count;
}

} // namespace planning
