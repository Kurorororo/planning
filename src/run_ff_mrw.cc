#include <string>

#include "ff.h"
#include "run_mrw.h"

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: ff <filename>" << std::endl;
    exit(1);
  }
  std::string filename = argv[1];
  mrw::Run<heuristic::FF>(filename);
}
