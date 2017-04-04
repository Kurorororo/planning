#include <string>

#include "goal_count.h"
#include "run_astar.h"

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: goal_count <filename>" << std::endl;
    exit(1);
  }
  std::string filename = argv[1];
  astar::Run<heuristic::GoalCount>(filename);
}
