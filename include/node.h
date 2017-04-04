#ifndef NODE_H_
#define NODE_H_

#include <vector>

#include <boost/intrusive_ptr.hpp>
#include <boost/pool/pool.hpp>

namespace node {

class Node {
 public:
  void Destroy() {
    this->~Node();
    node_pool.free(this);
  }

  std::string get_action() const {
    return action_;
  }

  void set_action(const std::string &action) {
    action_ = action;
  }

  int get_cost() const {
    return cost_;
  }

  void set_cost(int cost) {
    cost_ = cost;
  }

  int get_step() const {
    return step_;
  }

  void set_step(int step) {
    step_ = step;
  }

  boost::intrusive_ptr<Node> get_parent_node() {
    return parent_node_;
  }

  void set_parent_node(boost::intrusive_ptr<Node> parent_node) {
    parent_node_ = parent_node;
  }

  static boost::intrusive_ptr<Node> Construct() {
    return boost::intrusive_ptr<Node>(new(node_pool.malloc()) Node());
  }

  std::vector<int> variables_;

  static int n_node;

 private:
  Node() : action_(""), cost_(0), step_(0), parent_node_(nullptr) {}
  ~Node() {}

  int8_t ref_count;
  std::string action_;
  int cost_;
  int step_;
  boost::intrusive_ptr<Node> parent_node_;
  static boost::pool<> node_pool;

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

} // namespace node

#endif
