#include <iostream>
#include <string>

#include "ff.h"
#include "run_astar.h"

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: ff_astar <filename>" << std::endl;
    exit(1);
  }
  std::string filename = argv[1];
  planning::Run<planning::FF>(filename);
}
