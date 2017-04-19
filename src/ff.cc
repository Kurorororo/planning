#include "ff.h"

#include <vector>

#include "data.h"
#include "heuristic.h"

namespace planning {

int FF::operator()(const std::vector<int> &variables,
                   const std::vector<int> &fact_offset,
                   const std::vector<var_value_t> &goal,
                   const Actions &actions) {
  auto result = Search(variables, fact_offset, actions, schmea_, &graph_);
  int sum = 0;
  for (auto v : result) {
    if (v == -1) {
      return -1;
    }
    sum += actions.costs[v];
  }
  return sum;
}

} // planning
