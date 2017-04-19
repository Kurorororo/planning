#include "trie.h"

#include <algorithm>
#include <numeric>
#include <iostream>
#include <utility>
#include <vector>

#include "data.h"

namespace planning {

void TrieTable::Insert(int query, std::vector<var_value_t> precondition,
                       const std::vector<int> &fact_offset) {
  int i = 0;
  int j = 0;
  int max = precondition.size() - 1;
  int parent_prefix = 0;
  int max_children = fact_offset.back();
  if (a_.size() < max_children) {
    a_.resize(max_children);
    std::fill_n(a_.begin(), a_.size(), std::make_pair(-1, -1));
  }
  std::sort(precondition.begin(), precondition.end());
  for (auto v : precondition) {
    int var, value;
    DecodeVarValue(v, &var, &value);
    int index = j + fact_offset[var] - parent_prefix + value;
    if (i == max) {
      if (a_[index].second == -1) {
        a_[index].second = data_.size();
        data_.resize(data_.size()+1);
      }
      data_[a_[index].second].push_back(query);
      return;
    }
    parent_prefix = fact_offset[var+1];
    if (a_[index].first == -1) {
      a_[index].first = a_.size();
      a_.resize(a_.size()+max_children-parent_prefix);
      std::fill_n(&a_[a_[index].first], max_children-parent_prefix,
                  std::make_pair(-1, -1));
    }
    j = a_[index].first;
    ++i;
  }
}

std::vector<int> TrieTable::Find(const std::vector<int> &variables,
                                 const std::vector<int> &fact_offset) const {
  std::vector<int> result;
  RecursiveFind(variables, fact_offset, 0, 0, result);
  return std::move(result);
}

void TrieTable::RecursiveFind(const std::vector<int> &variables,
                              const std::vector<int> &fact_offset, int index,
                              int current, std::vector<int> &result) const {
  int prefix = index - fact_offset[current];
  for (int i=current, n=variables.size(); i<n; ++i) {
    int next = fact_offset[i] + variables[i] + prefix;
    int offset = a_[next].second;
    if (offset != -1)
      result.insert(result.end(), data_[offset].begin(), data_[offset].end());
    if (a_[next].first == -1) continue;
    RecursiveFind(variables, fact_offset, a_[next].first, i+1, result);
  }
}

TrieTable TrieTable::Construct(
    const std::vector< std::vector<var_value_t> > &preconditions,
    const std::vector<int> &fact_offset) {
  TrieTable table;
  int n = preconditions.size();
  for (int i=0; i<n; ++i) {
    table.Insert(i, preconditions[i], fact_offset);
  }
  return std::move(table);
}

void TrieTable::Print() const {
  for (auto v : a_) {
    std::cout << v.first << " ";
  }
  std::cout << std::endl;
  for (auto v : a_) {
    std::cout << v.second << " ";
  }
  std::cout << std::endl;
  for (auto v : data_) {
    for (auto u : v) {
      std::cout << u << " ";
    }
    std::cout << std::endl;
  }
}

} // namespace trie
