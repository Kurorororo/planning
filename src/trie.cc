#include "trie.h"

#include <numeric>
#include <map>
#include <iostream>
#include <queue>
#include <utility>
#include <vector>

namespace trie {

void TrieTable::set_code_table(const std::vector<int> &sups) {
  int n = sups.size();
  code_table_.resize(n);
  int sum = 0;
  for (int i=0; i<n; ++i) {
    code_table_[i] = sum;
    sum += sups[i];
  }
  max_children_ = sum;
  a_.resize(max_children_);
  std::fill_n(a_.begin(), a_.size(), std::make_pair(-1, -1));
}

void TrieTable::Insert(int query, const std::map<int, int> &precondition) {
  int i = 0;
  int j = 0;
  int max = precondition.size() - 1;
  int parent_prefix = 0;
  for (auto v : precondition) {
    int index = j + code_table_[v.first] - parent_prefix + v.second;
    if (i == max) {
      if (a_[index].second == -1) {
        a_[index].second = data_.size();
        data_.resize(data_.size()+1);
      }
      data_[a_[index].second].push_back(query);
      return;
    }
    parent_prefix = code_table_[v.first+1];
    if (a_[index].first == -1) {
      a_[index].first = a_.size();
      a_.resize(a_.size()+max_children_-parent_prefix);
      std::fill_n(&a_[a_[index].first], max_children_-parent_prefix,
                  std::make_pair(-1, -1));
    }
    j = a_[index].first;
    ++i;
  }
}

std::vector<int> TrieTable::Find(const std::vector<int> &variables) const {
  std::vector<int> result;
  std::queue< std::pair<int, int> > indexes;
  int n = variables.size();
  for (int i=0; i<n; ++i) {
    int index = code_table_[i] + variables[i];
    int offset = a_[index].second;
    if (a_[index].first != -1)
      indexes.push(std::make_pair(a_[index].first, i+1));
    if (offset != -1)
      result.insert(result.end(), data_[offset].begin(), data_[offset].end());
  }
  while (!indexes.empty()) {
    auto node = indexes.front();
    indexes.pop();
    int prefix = node.first - code_table_[node.second];
    for (int i=node.second; i<n; ++i) {
      int index = code_table_[i] + variables[i] + prefix;
      int offset = a_[index].second;
      if (a_[index].first != -1)
        indexes.push(std::make_pair(a_[index].first, i+1));
      if (offset != -1) {
        result.insert(result.end(), data_[offset].begin(),
                      data_[offset].end());
      }
    }
  }
  return std::move(result);
}

void SortbyPrecondition(const std::vector< std::vector<int> > &v,
                        std::vector<int> &indexes) {
  auto compare = [&v](int x, int y) -> int {
    for (int i=0, n=v[x].size(); i<n; ++i) {
      if (v[x][i] == v[y][i]) continue;
      return v[x][i] < v[y][i];
    }
    return x < y;
  };
  std::sort(indexes.begin(), indexes.end(), compare);
}

TrieTable TrieTable::Construct(
    const std::vector< std::map<int, int> > &preconditions,
    const std::vector<int> &sups) {
  TrieTable table;
  table.set_code_table(sups);
  int n = preconditions.size();
  // std::vector<int> indexes(n);
  // std::iota(indexes.begin(), indexes.end(), 0);
  // SortbyPrecondition(preconditions, indexes);
  for (int i=0; i<n; ++i) {
    // table.Insert(indexes[i], preconditions[indexes[i]], sups);
    table.Insert(i, preconditions[i]);
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
