#ifndef GOALCOUNT_H_
#define GOALCOUNT_H_

#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

#include "heuristic.h"
#include "sas_data.h"

namespace heuristic {

class Goalcount : public HeuristicInterface<Goalcount> {
 public:
  void Initialize(
     const std::vector<int> &sups,
     const std::unordered_map<int, int> &goal,
     const std::vector< std::map<int, int> > &preconditions,
     const std::vector< std::unique_ptr< sas_data::SASOperator> >
         &sas_operators) {
  }

  int operator()(
      const std::vector<int> &variables,
      const std::unordered_map<int, int> &goal,
      const std::vector< std::map<int, int> > &preconditions,
      const std::vector< std::unique_ptr< sas_data::SASOperator> >
          &sas_operators);
};

} // namespace heuristic

#endif // GOALCOUNT_H_
