#include "goalcount.h"

#include <unordered_map>
#include <vector>

#include "heuristic.h"

namespace heuristic {

int Goalcount::operator()(
  const std::vector<int> &variables,
  const std::unordered_map<int, int> &goal,
  const std::vector< std::map<int, int> > &preconditions,
  const std::vector< std::unique_ptr< sas_data::SASOperator> >
      &sas_operators) {
  int count = goal.size();
  for (auto v : goal)
    if (variables[v.first] == v.second) --count;
  return count;
}

} // namespace heuristic
