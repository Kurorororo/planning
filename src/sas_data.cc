#include "sas_data.h"

#include <vector>

namespace sas_data {

void Effect::ApplyEffect(std::vector<int> &variables) {
  if (!conditions_.empty()) {
    for (auto v : conditions_)
      if (variables[v.first] != v.second) return;
  }
  variables[var_] = value_;
}

void SASOperator::ApplyEffects(std::vector<int> &variables) {
  for (int i=0, n=effects_.size(); i<n; ++i)
    effects_[i]->ApplyEffect(variables);
}

bool GoalCheck(const std::vector<int> &variables,
               const std::unordered_map<int, int> &goal) {
  for (auto v : goal)
    if (variables[v.first] != v.second) return false;
  return true;
}

} // namespace sas_data
