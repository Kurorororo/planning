#include "graphplan.h"

#include <cassert>

#include <map>
#include <memory>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "sas_data.h"

using std::map;
using std::pair;
using std::queue;
using std::unique_ptr;
using std::unordered_map;
using std::unordered_set;
using std::vector;
using sas_data::SASOperator;

namespace graphplan {

void Graphplan::Initialize(const vector<int> &sups,
                           const unordered_map<int, int> &goal,
                           const vector< map<int, int> > &preconditions,
                           const vector< unique_ptr<SASOperator> >
                              &sas_operators) {
  InitializeFactLayers(sups);
  InitializeActionLayers(preconditions, sas_operators);
  TranslateGoal(goal);
}

void Graphplan::InitializeFactLayers(const vector<int> &sups) {
  int n = sups.size();
  fact_offset_.resize(n);
  int sum = 0;
  for (int i=0; i<n; ++i) {
    fact_offset_[i] = sum;
    sum += sups[i];
  }
  n_facts_ = sum;
  precondition_map_ = vector< vector<int> >(n_facts_);
  effect_map_ = vector< vector<int> >(n_facts_);
}

void Graphplan::InitializeActionLayers(
    const vector< map<int, int> > &preconditions,
    const vector< unique_ptr<SASOperator> > &sas_operators) {
  int n = preconditions.size();
  precondition_size_ = vector<int>(n, 0);
  for (int i=0; i<n; ++i) {
    precondition_size_[i] = preconditions[i].size();
    for (auto v : preconditions[i])
      precondition_map_[fact_offset_[v.first]+v.second].push_back(i);
    for (int j=0, n=sas_operators[i]->get_effcts().size(); j<n; ++j) {
      int var = sas_operators[i]->get_effcts()[j]->get_var();
      int value = sas_operators[i]->get_effcts()[j]->get_value();
      effect_map_[fact_offset_[var]+value].push_back(i);
    }
  }
  n_actions_ = n;
}

void Graphplan::TranslateGoal(const unordered_map<int, int> &goal) {
  n_goals_ = goal.size();
  for (auto v : goal)
    goal_facts_.insert(fact_offset_[v.first]+v.second);
}

vector<int> Graphplan::Search(
    const vector<int> &initial,
    const vector< map<int, int> > &preconditions,
    const vector< unique_ptr<SASOperator> > &sas_operators) {
  auto graph = ConstructGraph(initial, preconditions, sas_operators);
  return ExtractPlan(preconditions, sas_operators, graph);
}

PlanningGraph Graphplan::ConstructGraph(
    const vector<int> &initial,
    const vector< map<int, int> > &preconditions,
    const vector< unique_ptr<SASOperator> > &sas_operators) {
  queue< pair<int, int> > scheduled_facts;
  queue<int> scheduled_actions;
  PlanningGraph graph(n_facts_, n_actions_);
  for (int i=0, n=initial.size(); i<n; ++i)
    scheduled_facts.push(std::make_pair(i, initial[i]));
  int i = 0;
  while (true) {
    FactLayer(i, scheduled_facts, scheduled_actions, &graph);
    if (graph.found_goals.size() == n_goals_) {
      ++i;
      break;
    }
    ActionLayer(i, sas_operators, scheduled_actions, scheduled_facts, &graph);
    ++i;
  }
  graph.n_layers = i;
  return std::move(graph);
}

void Graphplan::FactLayer(
    int i,
    queue< pair<int, int> > &scheduled_facts,
    queue<int> &scheduled_actions, PlanningGraph* graph) {
  while (!scheduled_facts.empty()) {
    auto temp = scheduled_facts.front();
    scheduled_facts.pop();
    int index = fact_offset_[temp.first] + temp.second;
    graph->fact_layer_membership[index] = i;
    if (goal_facts_.find(index) != goal_facts_.end())
      graph->found_goals.insert(index);
    for (auto o : precondition_map_[index]) {
      if (++graph->precondition_counter[o] == precondition_size_[o])
        scheduled_actions.push(o);
    }
  }
}

void Graphplan::ActionLayer(
    int i,
    const vector< unique_ptr<SASOperator> > &sas_operators,
    queue<int> &scheduled_actions,
    queue< pair<int, int> > &scheduled_facts,
    PlanningGraph* graph) {
  while (!scheduled_actions.empty()) {
    int o = scheduled_actions.front();
    scheduled_actions.pop();
    graph->action_layer_membership[o] = i;
    for (int j=0, n=sas_operators[o]->get_effcts().size(); j<n; ++j) {
      int var = sas_operators[o]->get_effcts()[j]->get_var();
      int value = sas_operators[o]->get_effcts()[j]->get_value();
      if (graph->fact_layer_membership[fact_offset_[var]+value] == -1)
        scheduled_facts.push(std::make_pair(var, value));
    }
  }
}

vector<int> Graphplan::ExtractPlan(
    const vector< map<int, int> > &preconditions,
    const vector< unique_ptr<SASOperator> > &sas_operators,
    const PlanningGraph &graph) {
  int m = graph.n_layers - 1;
  vector<int> result;
  vector< unordered_set<int> > g(graph.n_layers);
  vector< unordered_set<int> > marked(n_facts_);
  for (auto v : goal_facts_)
    g[graph.fact_layer_membership[v]].insert(v);
  for (int i=m; i>0; --i) {
    for (auto v : g[i]) {
      if (marked[v].find(i) != marked[v].end()) continue;
      int o = ChooseAction(v, i, preconditions, graph);
      for (auto f : preconditions[o]) {
        int index = fact_offset_[f.first] + f.second;
        int j = graph.fact_layer_membership[index];
        if (j != 0 && marked[v].find(i-1) == marked[v].end())
          g[j].insert(index);
      }
      for (int j=0, n=sas_operators[o]->get_effcts().size(); j<n; ++j) {
        int var = sas_operators[o]->get_effcts()[j]->get_var();
        int value = sas_operators[o]->get_effcts()[j]->get_value();
        int f_index = fact_offset_[var] + value;
        marked[f_index].insert(i);
        marked[f_index].insert(i-1);
      }
      result.push_back(o);
    }
  }
  return std::move(result);
}

int Graphplan::ChooseAction(int index, int i,
                            const vector< map<int, int> > &preconditions,
                            const PlanningGraph &graph) {
  int min = -1;
  int argmin = 0;
  for (auto o : effect_map_[index]) {
    if (graph.action_layer_membership[o] != i-1) continue;
    int difficulty = 0;
    for (auto p : preconditions[o]) {
      difficulty +=
          graph.fact_layer_membership[fact_offset_[p.first]+p.second];
    }
    if (difficulty < min || min == -1) {
      min = difficulty;
      argmin = o;
    }
  }
  assert(-1 != min);
  return argmin;
}

} // namespace graphplan
