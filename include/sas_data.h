#ifndef SAS_DATA_H_
#define SAS_DATA_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace sas_data {

class Effect {
 public:
  Effect() {}
  ~Effect() {}

  int get_var() const {
    return var_;
  }

  int get_value() const {
    return value_;
  }

  void set_effect(int var, int value) {
    var_ = var;
    value_ = value;
  }

  void push_condition(int var, int value) {
    conditions_[var] = value;
  }

  void ApplyEffect(std::vector<int> &variables);

  static std::unique_ptr<Effect> Create() {
    return std::move(std::unique_ptr<Effect>(new Effect()));
  }

 private:
  int var_;
  int value_;
  std::unordered_map<int, int> conditions_;
};

class SASOperator {
 public:
  SASOperator() {}
  ~SASOperator() {}

  int get_cost() const {
    return cost_;
  }

  void set_cost(const int cost) {
    cost_ = cost;
  }

  std::string get_name() const {
    return name_;
  }

  void set_name(const std::string &name) {
    name_ = name;
  }

  std::vector< std::unique_ptr<Effect> >& get_effcts() {
    return effects_;
  }

  void ApplyEffects(std::vector<int> &variables);

  static std::unique_ptr<SASOperator> Create() {
    return std::move(std::unique_ptr<SASOperator>(new SASOperator()));
  }

 private:
  int cost_;
  std::string name_;
  std::vector< std::unique_ptr<Effect> > effects_;
};

bool GoalCheck(const std::vector<int> &variables,
               const std::unordered_map<int, int> &goal);

} // namespace sas_data

#endif // SAS_DATA_H_
