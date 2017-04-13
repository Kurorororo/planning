#include <iostream>
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

#include "graphplan.h"
#include "parser.h"

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: graphplan <filename>" << std::endl;
    exit(1);
  }
  std::string filename = argv[1];
  std::vector<int> initial;
  std::vector<int> sups;
  std::vector< std::unordered_map<int, int> > mutex_groups;
  std::unordered_map<int, int> goal;
  std::vector< std::map<int, int> > preconditions;
  std::vector< std::unique_ptr<sas_data::SASOperator> > sas_operators;
  parser::Parse(filename, initial, sups, mutex_groups, goal, preconditions,
                sas_operators);
  graphplan::Graphplan plan(sups, goal, preconditions, sas_operators);
  auto result = plan.Search(initial, preconditions, sas_operators);
  for (int i=result.size()-1; i>-1; --i) {
    if (result[i] == -1) {
      std::cout << "faild to solve problem." << std::endl;
      exit(0);
    }
    std::cout << sas_operators[result[i]]->get_name() << std::endl;
  }
}
