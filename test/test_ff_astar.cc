#include <iostream>
#include <string>

#include "ff.h"
#include "test_astar.h"

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: test_ff_astar <filename>" << std::endl;
    exit(1);
  }
  std::string filename = argv[1];
  Test<planning::FF>(filename);
}
