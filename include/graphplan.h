#ifndef GRAPHPLAN_H_
#define GRAPHPLAN_H_

#include <map>
#include <memory>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "sas_data.h"

namespace graphplan {

struct PlanningGraph {
  int n_layers;
  std::vector<int> fact_layer_membership;
  std::vector<int> action_layer_membership;
  std::vector<int> precondition_counter;
  std::unordered_set<int> found_goals;

  PlanningGraph(int n_facts, int n_actions) {
    n_layers = 0;
    fact_layer_membership = std::vector<int>(n_facts, -1);
    action_layer_membership = std::vector<int>(n_actions, -1);
    precondition_counter = std::vector<int>(n_actions, 0);
  }

  ~PlanningGraph() {}
};

class Graphplan {
 public:
  Graphplan(
      const std::vector<int> &sups,
      const std::unordered_map<int, int> &goal,
      const std::vector< std::map<int, int> > &preconditions,
      const std::vector< std::unique_ptr< sas_data::SASOperator> >
          &sas_operators) {
    Initialize(sups, goal, preconditions, sas_operators);
  }

  ~Graphplan() {}

  std::vector<int> Search(
      const std::vector<int> &initial,
      const std::vector< std::map<int, int> > &preconditions,
      const std::vector< std::unique_ptr<sas_data::SASOperator> >
          &sas_operators);

 private:
  void Initialize(
      const std::vector<int> &sups,
      const std::unordered_map<int, int> &goal,
      const std::vector< std::map<int, int> > &preconditions,
      const std::vector< std::unique_ptr< sas_data::SASOperator> >
          &sas_operators);
  void InitializeFactLayers(const std::vector<int> &sups);
  void InitializeActionLayers(
      const std::vector< std::map<int, int> > &preconditions,
      const std::vector< std::unique_ptr<sas_data::SASOperator> >
          &sas_operators);
  void TranslateGoal(const std::unordered_map<int, int> &goal);
  PlanningGraph ConstructGraph(
      const std::vector<int> &initial,
      const std::vector< std::map<int, int> > &preconditions,
      const std::vector< std::unique_ptr< sas_data::SASOperator> >
          &sas_operators);
  void FactLayer(int i, std::queue< std::pair<int, int> > &scheduled_facts,
                 std::queue<int> &scheduled_actions, PlanningGraph* graph);
  void ActionLayer(
      int i,
      const std::vector< std::unique_ptr<sas_data::SASOperator> >
          &sas_operators,
      std::queue<int> &scheduled_actions,
      std::queue< std::pair<int, int> > &scheduled_facts,
      PlanningGraph* graph);
  std::vector<int> ExtractPlan(
     const std::vector< std::map<int, int> > &preconditions,
     const std::vector< std::unique_ptr<sas_data::SASOperator> >
        &sas_operators,
     const PlanningGraph &graph);
  int ChooseAction(int index, int i,
                   const std::vector< std::map<int, int> > &preconditions,
                   const PlanningGraph &graph);

  int n_facts_;
  int n_actions_;
  int n_goals_;
  std::vector<int> fact_offset_;
  std::vector<int> precondition_size_;
  std::vector< std::vector<int> > precondition_map_;
  std::vector< std::vector<int> > effect_map_;
  std::unordered_set<int> goal_facts_;
};

} // namespace graphplan

#endif // GRAPHPLAN_H_
