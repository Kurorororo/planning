#ifndef TRIE_H_
#define TRIE_H_

#include <vector>

namespace trie {

class TrieTable {
 public:
  TrieTable(int n) {
    a_.resize(n+1, -1);
  }

  ~TrieTable() {}

  void Insert(int query, const std::vector<int> &preconditions,
              const std::vector<int> &sups);
  void Find(const std::vector<int> &variables, std::vector<int> &result) const;

  void Print() const;

  static TrieTable Construct(
      const std::vector< std::vector<int> > &preconditions,
      const std::vector<int> &sups);

 private:
  std::vector<int> a_;
  std::vector< std::vector<int> > data_;
};

} // namespace trie

#endif
