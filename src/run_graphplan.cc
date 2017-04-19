#include <iostream>
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

#include "data.h"
#include "graphplan.h"
#include "parser.h"

using planning::var_value_t;

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: graphplan <filename>" << std::endl;
    exit(1);
  }
  std::string filename = argv[1];
  std::vector<int> initial;
  std::vector<int> fact_offset;
  std::vector< std::vector<var_value_t> > mutex_groups;
  std::vector<var_value_t> goal;
  planning::Actions actions;
  planning::Parse(filename, initial, fact_offset, mutex_groups, goal,
                  &actions);
  planning::GraphSchema schema;
  planning::InitializeSchema(fact_offset, goal, actions, &schema);
  planning::PlanningGraph graph;
  planning::InitializeGraph(fact_offset, schema, &graph);
  auto result = Search(initial, fact_offset, actions, schema, &graph);
  for (int i=result.size()-1; i>-1; --i) {
    if (result[i] == -1) {
      std::cout << "faild to solve problem." << std::endl;
      exit(0);
    }
    std::cout << actions.names[result[i]] << std::endl;
  }
}
