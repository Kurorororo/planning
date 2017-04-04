#ifndef HEURISTIC_NODE_H_
#define HEURISTIC_NODE_H_

#include <cstdint>

#include <vector>

#include <boost/intrusive_ptr.hpp>
#include <boost/pool/pool.hpp>

namespace heuristic_node {

class HeuristicNode {
 public:
  void Destroy() {
    this->~HeuristicNode();
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

  int get_heuristic() const {
    return heuristic_;
  }

  void set_heuristic(int heuristic) {
    heuristic_ = heuristic;
  }

  int get_step() const {
    return step_;
  }

  void set_step(int step) {
    step_ = step;
  }

  boost::intrusive_ptr<HeuristicNode> get_parent_node() {
    return parent_node_;
  }

  void set_parent_node(boost::intrusive_ptr<HeuristicNode> parent_node) {
    parent_node_ = parent_node;
  }

  static boost::intrusive_ptr<HeuristicNode> Construct() {
    return boost::intrusive_ptr<HeuristicNode>(
        new(node_pool.malloc()) HeuristicNode());
  }

  std::vector<int> variables_;

  static int n_node;

 private:
  HeuristicNode()
      : action_(""),
        cost_(0),
        heuristic_(0),
        step_(0),
        parent_node_(nullptr) {
  }

  ~HeuristicNode() {}

  int8_t ref_count;
  std::string action_;
  int cost_;
  int heuristic_;
  int step_;
  boost::intrusive_ptr<HeuristicNode> parent_node_;
  static boost::pool<> node_pool;

  friend void intrusive_ptr_add_ref(HeuristicNode* ptr) {
    ++(ptr->ref_count);
  }

  friend void intrusive_ptr_release(HeuristicNode* ptr) {
    if (--(ptr->ref_count) <= 0) {
      ptr->Destroy();
    }
  }
};

boost::pool<> HeuristicNode::node_pool(sizeof(HeuristicNode), 100000);

using ptr_t = boost::intrusive_ptr<HeuristicNode>;

} // namespace heuristic_node

#endif
