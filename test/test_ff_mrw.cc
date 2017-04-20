#include <iostream>
#include <string>

#include "ff.h"
#include "test_mrw.h"

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: test_ff_mrw <filename>" << std::endl;
    exit(1);
  }
  std::string filename = argv[1];
  Test<planning::FF>(filename);
}
