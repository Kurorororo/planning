#include "trie.h"

#include <numeric>
#include <iostream>
#include <queue>
#include <utility>
#include <vector>

namespace trie {

void TrieTable::Insert(int query, const std::vector<int> &precondition,
                       const std::vector<int> &sups) {
  int j = 0;
  for (int i=0, n=precondition.size(); i<n; ++i) {
    int v = precondition[i];
    if (i == n-1) {
      if (a_[j+v+1] == -1) {
        a_[j+v+1] = data_.size();
        data_.resize(data_.size()+1);
      }
      data_[a_[j+v+1]].push_back(query);
      return;
    }
    if (a_[j+v+1] == -1) {
      a_[j+v+1] = a_.size();
      a_.resize(a_.size()+sups[i+1]+1);
      std::fill_n(&a_[a_[j+v+1]], sups[i+1]+1, -1);
    }
    j = a_[j+v+1];
  }
}

void TrieTable::Find(const std::vector<int> &variables,
                     std::vector<int> &result) const {
  std::queue< std::pair<int, int> > indexes;
  if (a_[0] != -1)
    indexes.push(std::make_pair(0, a_[0]));
  if (a_[variables[0]+1] != -1)
    indexes.push(std::make_pair(0, a_[variables[0]+1]));
  int max_depth = variables.size();
  while (!indexes.empty()) {
    auto node = indexes.front();
    int depth = node.first + 1;
    int offset = node.second;
    indexes.pop();
    if (depth == max_depth) {
      result.insert(result.end(), data_[offset].begin(), data_[offset].end());
      continue;
    }
    int next = offset + variables[depth] + 1;
    if (a_[offset] != -1)
      indexes.push(std::make_pair(depth, a_[offset]));
    if (a_[next] != -1)
      indexes.push(std::make_pair(depth, a_[next]));
  }
}

void TrieTable::Print() const {
  for (auto v : a_) {
    std::cout << v << " ";
  }
  std::cout << std::endl;
  for (auto v : data_) {
    for (auto u : v) {
      std::cout << u << " ";
    }
    std::cout << std::endl;
  }
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
    const std::vector< std::vector<int> > &preconditions,
    const std::vector<int> &sups) {
  TrieTable table(sups[0]);
  int n = preconditions.size();
  // std::vector<int> indexes(n);
  // std::iota(indexes.begin(), indexes.end(), 0);
  // SortbyPrecondition(preconditions, indexes);
  for (int i=0; i<n; ++i) {
    // table.Insert(indexes[i], preconditions[indexes[i]], sups);
    table.Insert(i, preconditions[i], sups);
  }
  return std::move(table);
}

} // namespace trie
