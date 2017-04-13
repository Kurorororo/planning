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

#include <iostream>

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
    const vector< unique_ptr<SASOperator> > &sas_operators) const {
  auto graph = ConstructGraph(initial, preconditions, sas_operators);
  if (graph.n_layers == -1)
    return std::vector<int>{-1};
  return ExtractPlan(preconditions, sas_operators, graph);
}

PlanningGraph Graphplan::ConstructGraph(
    const vector<int> &initial,
    const vector< map<int, int> > &preconditions,
    const vector< unique_ptr<SASOperator> > &sas_operators) const {
  queue<int> scheduled_facts;
  queue<int> scheduled_actions;
  PlanningGraph graph(n_facts_, n_actions_);
  for (int i=0, n=initial.size(); i<n; ++i) {
    int f = fact_offset_[i]+initial[i];
    graph.closed.insert(f);
    scheduled_facts.push(f);
  }
  int i = 0;
  while (true) {
    if (scheduled_facts.empty()) {
      graph.n_layers = -1;
      // for (int j=0; j<precondition_size_.size(); ++j) {
      //   std::cout << graph.precondition_counter[j] << "(" << precondition_size_[j] << "){ ";
      //   for (auto v : preconditions[j]) {
      //     std::cout << graph.fact_layer_membership[fact_offset_[v.first]+v.second] << " ";
      //   }
      //   std::cout << "}" << std::endl;
      // }
      // std::cout << "initial" << std::endl;
      // for (int j=0; j<initial.size(); ++j) {
      //   std::cout << "var" << j << " " << initial[j] << std::endl;
      // }
      // std::cout << "dead end " << i << std::endl;
      // for (int j=0; j<fact_offset_.size()-1; ++j) {
      //   std::cout << "var" << j;
      //   for (int k=0; k<fact_offset_[j+1]-fact_offset_[j]; ++k) {
      //     if (graph.fact_layer_membership[fact_offset_[j]+k] != -1)
      //       std::cout << " " << k;
      //   }
      //   std::cout << std::endl;
      // }
      // int j = fact_offset_.size()-1;
      // std::cout << "var" << j;
      // for (int k=0; k<graph.fact_layer_membership.size()-fact_offset_[j]; ++k) {
      //   if (graph.fact_layer_membership[fact_offset_[j]+k] != -1)
      //     std::cout << " " << k;
      // }
      // std::cout << std::endl;
      // exit(0);
      return std::move(graph);
    }
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
    queue<int> &scheduled_facts,
    queue<int> &scheduled_actions, PlanningGraph* graph) const {
  while (!scheduled_facts.empty()) {
    int f = scheduled_facts.front();
    scheduled_facts.pop();
    graph->fact_layer_membership[f] = i;
    if (goal_facts_.find(f) != goal_facts_.end())
      graph->found_goals.insert(f);
    for (auto o : precondition_map_[f]) {
      if (++graph->precondition_counter[o] == precondition_size_[o])
        scheduled_actions.push(o);
    }
  }
}

void Graphplan::ActionLayer(
    int i,
    const vector< unique_ptr<SASOperator> > &sas_operators,
    queue<int> &scheduled_actions,
    queue<int> &scheduled_facts,
    PlanningGraph* graph) const {
  while (!scheduled_actions.empty()) {
    int o = scheduled_actions.front();
    scheduled_actions.pop();
    graph->action_layer_membership[o] = i;
    for (int j=0, n=sas_operators[o]->get_effcts().size(); j<n; ++j) {
      int var = sas_operators[o]->get_effcts()[j]->get_var();
      int value = sas_operators[o]->get_effcts()[j]->get_value();
      int f = fact_offset_[var] + value;
      if (graph->closed.find(f) == graph->closed.end()) {
        graph->closed.insert(f);
        scheduled_facts.push(f);
      }
    }
  }
}

vector<int> Graphplan::ExtractPlan(
    const vector< map<int, int> > &preconditions,
    const vector< unique_ptr<SASOperator> > &sas_operators,
    const PlanningGraph &graph) const {
  int m = graph.n_layers - 1;
  vector<int> result;
  vector< unordered_set<int> > g_set(graph.n_layers);
  vector< unordered_set<int> > marked(n_facts_);
  for (auto g : goal_facts_)
    g_set[graph.fact_layer_membership[g]].insert(g);
  for (int i=m; i>0; --i) {
    for (auto g : g_set[i]) {
      if (marked[g].find(i) != marked[g].end()) continue;
      int o = ChooseAction(g, i, preconditions, graph);
      for (auto v : preconditions[o]) {
        int f = fact_offset_[v.first] + v.second;
        int j = graph.fact_layer_membership[f];
        if (j != 0 && marked[f].find(i-1) == marked[f].end())
          g_set[j].insert(f);
      }
      for (int j=0, n=sas_operators[o]->get_effcts().size(); j<n; ++j) {
        int var = sas_operators[o]->get_effcts()[j]->get_var();
        int value = sas_operators[o]->get_effcts()[j]->get_value();
        int f = fact_offset_[var] + value;
        marked[f].insert(i);
        marked[f].insert(i-1);
      }
      result.push_back(o);
    }
  }
  return std::move(result);
}

int Graphplan::ChooseAction(int index, int i,
                            const vector< map<int, int> > &preconditions,
                            const PlanningGraph &graph) const {
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
