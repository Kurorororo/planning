#include <iostream>
#include <string>
#include <vector>

#include "mrw.h"
#include "parser.h"
#include "trie.h"

using planning::var_value_t;

template<class T>
void Test(std::string filename) {
  std::vector<int> initial;
  std::vector<int> fact_offset;
  std::vector< std::vector<var_value_t> > mutex_groups;
  std::vector<var_value_t> goal;
  planning::Actions actions;

  planning::Parse(filename, initial, fact_offset, mutex_groups, goal,
                  &actions);

  auto table = planning::TrieTable::Construct(actions.preconditions,
                                              fact_offset);
  auto result = planning::MRW<T>(initial, fact_offset, goal, actions, table);

  if (result == nullptr) {
    std::cout << "faild to solve problem" << std::endl;
    exit(0);
  }
  std::cout << "plan" << std::endl;

  std::vector<std::string> name_array;
  std::vector<int> result_array;
  while (result->parent_node != nullptr) {
    name_array.push_back(actions.names[result->action]);
    result_array.insert(result_array.begin(), result->action);
    result = result->parent_node;
  }

  while (!name_array.empty()) {
    std::cout << name_array.back() << std::endl;
    name_array.pop_back();
  }

  std::cout << "test" << std::endl;

  auto variables = initial;
  for (int i=0; i<result_array.size(); ++i) {
    int o = result_array[i];
    for (auto f : actions.preconditions[o]) {
      int var, value;
      planning::DecodeVarValue(f, &var, &value);
      if (variables[var] != value) {
        std::cerr << "layer" << i << " precondition var" << var << "=" << value
                  << " is not satisfied for action " << actions.names[o]
                  << std::endl;
        exit(1);
      }
    }
    for (auto f : actions.effects[o]) {
      int var, value;
      planning::DecodeVarValue(f, &var, &value);
      variables[var] = value;
    }
  }
  for (auto g : goal) {
    int var, value;
    planning::DecodeVarValue(g, &var, &value);
    if (variables[var] != value) {
      std::cerr << "goal var" << var << "=" << value << " is not satisfied"
                << std::endl;
      exit(1);
    }
  }
  std::cout << "OK!" << std::endl;
}
