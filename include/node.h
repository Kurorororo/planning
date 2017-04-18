#ifndef NODE_H_
#define NODE_H_

#include <vector>

#include <boost/intrusive_ptr.hpp>
#include <boost/pool/pool.hpp>

namespace planning {

struct Node {
  int action;
  int g;
  int8_t ref_count;
  boost::intrusive_ptr<Node> parent_node;
  std::vector<int> variables;

  static boost::pool<> node_pool;

  Node() : action(-1), g(0), ref_count(0), parent_node(nullptr) {}
  ~Node() {}

  void Destroy() {
    this->~Node();
    node_pool.free(this);
  }

  static boost::intrusive_ptr<Node> Construct() {
    return boost::intrusive_ptr<Node>(new(node_pool.malloc()) Node());
  }

  friend void intrusive_ptr_add_ref(Node* ptr) {
    ++(ptr->ref_count);
  }

  friend void intrusive_ptr_release(Node* ptr) {
    if (--(ptr->ref_count) <= 0) {
      ptr->Destroy();
    }
  }
};

boost::pool<> Node::node_pool(sizeof(Node), 100000);

using ptr_t = boost::intrusive_ptr<Node>;

} // namespace node

#endif // NODE_H_
