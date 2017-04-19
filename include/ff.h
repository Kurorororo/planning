#ifndef FF_H_
#define FF_H_

#include <vector>

#include "data.h"
#include "graphplan.h"
#include "heuristic.h"

namespace planning {

class FF : public HeuristicInterface<FF> {
 public:
  void Initialize(const std::vector<int> &fact_offset,
                  const std::vector<var_value_t> &goal,
                  const Actions &actions) {
    InitializeSchema(fact_offset, goal, actions, &schmea_);
    InitializeGraph(fact_offset, schmea_, &graph_);
  }

  int operator()(const std::vector<int> &variables,
                 const std::vector<int> &fact_offset,
                 const std::vector<var_value_t> &goal,
                 const Actions &actions);

 private:
  GraphSchema schmea_;
  PlanningGraph graph_;
};

} // namespace heuristic

#endif // GOALCOUNT_H_
