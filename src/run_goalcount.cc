#include <string>

#include "goalcount.h"
#include "run_astar.h"

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: goalcount <filename>" << std::endl;
    exit(1);
  }
  std::string filename = argv[1];
  astar::Run<heuristic::Goalcount>(filename);
}
