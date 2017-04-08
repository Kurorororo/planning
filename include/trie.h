#ifndef TRIE_H_
#define TRIE_H_

#include <map>
#include <vector>

namespace trie {

class TrieTable {
 public:
  TrieTable() {}
  ~TrieTable() {}

  void set_code_table(const std::vector<int> &sups);
  void Insert(int query, const std::map<int, int> &precondition);
  std::vector<int> Find(const std::vector<int> &variables) const;

  void Print() const;

  static TrieTable Construct(
      const std::vector< std::map<int, int> > &preconditions,
      const std::vector<int> &sups);

 private:
  int max_children_;
  std::vector< std::pair<int, int> > a_;
  std::vector<int> code_table_;
  std::vector< std::vector<int> > data_;
};

} // namespace trie

#endif // TRIE_H_
