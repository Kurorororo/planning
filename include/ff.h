#ifndef GOALCOUNT_H_
#define GOALCOUNT_H_

#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

#include "graphplan.h"
#include "heuristic.h"
#include "sas_data.h"

#include <iostream>

namespace heuristic {

class FF : public HeuristicInterface<FF> {
 public:
  void Initialize(
    const std::vector<int> &sups,
    const std::unordered_map<int, int> &goal,
    const std::vector< std::map<int, int> > &preconditions,
    const std::vector< std::unique_ptr< sas_data::SASOperator> >
        &sas_operators) {
    planner_ = graphplan::Graphplan(sups, goal, preconditions, sas_operators);
  }

  int operator()(
      const std::vector<int> &variables,
      const std::unordered_map<int, int> &goal,
      const std::vector< std::map<int, int> > &preconditions,
      const std::vector< std::unique_ptr< sas_data::SASOperator> >
          &sas_operators) {
    auto result = planner_.Search(variables, preconditions, sas_operators);
    int sum = 0;
    for (auto o : result) {
      if (o == -1) return -1;
      sum += sas_operators[o]->get_cost();
    }
    return sum;
  }

 private:
  graphplan::Graphplan planner_;
};

} // namespace heuristic

#endif // GOALCOUNT_H_
