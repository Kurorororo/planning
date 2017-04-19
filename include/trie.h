#ifndef TRIE_H_
#define TRIE_H_

#include <utility>
#include <vector>

#include "data.h"

namespace planning {

class TrieTable {
 public:
  TrieTable() {}
  ~TrieTable() {}

  void Insert(int query, std::vector<var_value_t> precondition,
              const std::vector<int> &fact_offset);
  std::vector<int> Find(const std::vector<int> &variables,
                        const std::vector<int> &fact_offset) const;

  void Print() const;

  static TrieTable Construct(
      const std::vector< std::vector<var_value_t> > &preconditions,
      const std::vector<int> &sups);

 private:
  void RecursiveFind(const std::vector<int> &variables,
                     const std::vector<int> &fact_offset, int index,
                     int current, std::vector<int> &result) const;

  std::vector< std::pair<int, int> > a_;
  std::vector< std::vector<int> > data_;
};

} // namespace planning

#endif // TRIE_H_
