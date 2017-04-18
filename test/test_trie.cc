#include "trie.h"

#include <cassert>

#include <iostream>
#include <vector>

#include "data.h"

using planning::TrieTable;
using planning::var_value_t;
using planning::EncodeVarValue;

void TestInsert() {
  std::cout << "Started test Trie::Insert()" << std::endl;

  std::vector<int> fact_offset{0, 2, 5, 10, 12};
  TrieTable table;
  std::vector<var_value_t> precondition(4);
  EncodeVarValue(0, 1, &precondition[0]);
  EncodeVarValue(1, 2, &precondition[1]);
  EncodeVarValue(2, 4, &precondition[2]);
  EncodeVarValue(3, 1, &precondition[3]);
  table.Insert(0, precondition, fact_offset);

  std::cout << "Passed all test cases" << std::endl;
}

void TestFind() {
  std::cout << "Started test Trie::Find()" << std::endl;

  std::vector<int> fact_offset{0, 2, 5, 10, 12};
  TrieTable table;

  std::vector<var_value_t> precondition(3);
  EncodeVarValue(0, 1, &precondition[0]);
  EncodeVarValue(2, 4, &precondition[1]);
  EncodeVarValue(3, 1, &precondition[2]);
  table.Insert(0, precondition, fact_offset);

  std::vector<int> variables{1, 2, 4, 1};
  auto result = table.Find(variables, fact_offset);
  assert(1 == result.size());
  assert(0 == result[0]);
  variables[1] = 0;
  result = table.Find(variables, fact_offset);
  assert(1 == result.size());
  assert(0 == result[0]);
  var_value_t var_value;
  EncodeVarValue(1, 0, &var_value);
  precondition.push_back(var_value);
  table.Insert(1, precondition, fact_offset);
  result = table.Find(variables, fact_offset);
  assert(2 == result.size());
  if (result[0] == 0) {
    assert(1 == result[1]);
  } else {
    assert(1 == result[0]);
    assert(0 == result[1]);
  }
  variables[1] = 2;
  result = table.Find(variables, fact_offset);
  assert(1 == result.size());
  assert(0 == result[0]);
  variables[2] = 2;
  result = table.Find(variables, fact_offset);
  assert(0 == result.size());

  std::cout << "Passed all test cases" << std::endl;
}

void TestConstruct() {
  std::cout << "Started test Trie::Construct()" << std::endl;

  std::vector<int> fact_offset{0, 2, 5, 10, 12};
  std::vector< std::vector<var_value_t> > preconditions(2);
  var_value_t var_value;
  EncodeVarValue(0, 1, &var_value);
  preconditions[0].push_back(var_value);
  EncodeVarValue(2, 4, &var_value);
  preconditions[0].push_back(var_value);
  EncodeVarValue(3, 1, &var_value);
  preconditions[0].push_back(var_value);
  EncodeVarValue(0, 1, &var_value);
  preconditions[1].push_back(var_value);
  EncodeVarValue(1, 2, &var_value);
  preconditions[1].push_back(var_value);
  EncodeVarValue(2, 4, &var_value);
  preconditions[1].push_back(var_value);
  EncodeVarValue(3, 1, &var_value);
  preconditions[1].push_back(var_value);
  auto table = TrieTable::Construct(preconditions, fact_offset);
  std::vector<int> variables{1, 2, 4, 1};
  auto result = table.Find(variables, fact_offset);
  assert(2 == result.size());
  variables[1] = 0;
  result = table.Find(variables, fact_offset);
  assert(1 == result.size());
  variables[2] = 2;
  result = table.Find(variables, fact_offset);
  assert(0 == result.size());

  std::cout << "Passed all test cases" << std::endl;
}

int main() {
  TestInsert();
  TestFind();
  TestConstruct();
}
