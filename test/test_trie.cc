#include "trie.h"

#include <cassert>

#include <iostream>
#include <map>
#include <vector>

using trie::TrieTable;

void TestSet_code_table() {
  std::cout << "Started test Trie::set_code_table()" << std::endl;

  std::vector<int> sups{2, 3, 5, 2};
  TrieTable table;
  table.set_code_table(sups);

  std::cout << "Passed all test cases" << std::endl;
}

void TestInsert() {
  std::cout << "Started test Trie::Insert()" << std::endl;

  std::vector<int> sups{2, 3, 5, 2};
  TrieTable table;
  table.set_code_table(sups);
  std::map<int, int> precondition;
  precondition[0] = 1;
  precondition[1] = 2;
  precondition[2] = 4;
  precondition[3] = 1;
  table.Insert(0, precondition);

  std::cout << "Passed all test cases" << std::endl;
}

void TestFind() {
  std::cout << "Started test Trie::Find()" << std::endl;

  std::vector<int> sups{2, 3, 5, 2};
  TrieTable table;
  table.set_code_table(sups);
  std::map<int, int> precondition;
  precondition[0] = 1;
  precondition[2] = 4;
  precondition[3] = 1;
  table.Insert(0, precondition);
  std::vector<int> variables{1, 2, 4, 1};
  auto result = table.Find(variables);
  assert(1 == result.size());
  assert(0 == result[0]);
  variables[1] = 0;
  result = table.Find(variables);
  assert(1 == result.size());
  assert(0 == result[0]);
  precondition[1] = 0;
  table.Insert(1, precondition);
  result = table.Find(variables);
  assert(2 == result.size());
  if (variables[0] == 0) {
    assert(1 == variables[1]);
  } else {
    assert(1 == variables[0]);
    assert(0 == variables[1]);
  }
  variables[1] = 2;
  result = table.Find(variables);
  assert(1 == result.size());
  assert(0 == result[0]);
  variables[2] = 2;
  result = table.Find(variables);
  assert(0 == result.size());

  std::cout << "Passed all test cases" << std::endl;
}

void TestConstruct() {
  std::cout << "Started test Trie::Construct()" << std::endl;

  std::vector<int> sups{2, 3, 5, 2};
  std::vector< std::map<int, int> > preconditions(2);
  preconditions[0][0] = 1;
  preconditions[0][2] = 4;
  preconditions[0][3] = 1;
  preconditions[1][0] = 1;
  preconditions[1][1] = 2;
  preconditions[1][2] = 4;
  preconditions[1][3] = 1;
  auto table = TrieTable::Construct(preconditions, sups);
  std::vector<int> variables{1, 2, 4, 1};
  auto result = table.Find(variables);
  assert(2 == result.size());
  variables[1] = 0;
  result = table.Find(variables);
  assert(1 == result.size());
  variables[2] = 2;
  result = table.Find(variables);
  assert(0 == result.size());

  std::cout << "Passed all test cases" << std::endl;
}

int main() {
  TestSet_code_table();
  TestInsert();
  TestFind();
  TestConstruct();
}
