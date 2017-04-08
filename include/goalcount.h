#ifndef GOALCOUNT_H_
#define GOALCOUNT_H_

#include <unordered_map>
#include <vector>

#include "heuristic.h"

namespace heuristic {

class Goalcount : public HeuristicInterface<Goalcount> {
 public:
  int operator()(const std::vector<int> &variables,
                 const std::unordered_map<int, int> &goal);
};

} // namespace heuristic

#endif // GOALCOUNT_H_
