#ifndef GOALCOUNT_H_
#define GOALCOUNT_H_

#include <vector>

#include "data.h"
#include "heuristic.h"

namespace planning {

class Goalcount : public HeuristicInterface<Goalcount> {
 public:
  void Initialize(const std::vector<int> &fact_offset,
                  const std::vector<var_value_t> &goal,
                  const Actions &actions) {
  }

  int operator()(const std::vector<int> &variables,
                 const std::vector<int> &fact_offset,
                 const std::vector<var_value_t> &goal,
                 const Actions &actions);
};

} // namespace planning

#endif // GOALCOUNT_H_
