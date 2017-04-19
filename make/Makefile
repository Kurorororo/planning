SRC_DIR=../src
INC_DIR=../include
TEST_DIR=../test
BIN_DIR=../bin

INCS = -I$(INC_DIR) -I/usr/local/include/

UNAME := $(shell uname -s)
ifeq ($(UNAME),Linux)
  CXX=g++
endif
ifeq ($(UNAME),Darwin)
  CXX=/usr/bin/clang++
endif

goalcount_astar:
	$(CXX) -std=c++14 -O3 $(INCS) $(SRC_DIR)/run_goalcount_astar.cc \
  $(SRC_DIR)/goalcount.cc $(SRC_DIR)/data.cc $(SRC_DIR)/trie.cc \
  $(SRC_DIR)/parser.cc -o $(BIN_DIR)/goalcount_astar

ff_astar:
	$(CXX) -std=c++14 -O3 $(INCS) $(SRC_DIR)/run_ff_astar.cc $(SRC_DIR)/ff.cc  \
  $(SRC_DIR)/data.cc $(SRC_DIR)/trie.cc $(SRC_DIR)/parser.cc \
  $(SRC_DIR)/graphplan.cc -o $(BIN_DIR)/ff_astar

goalcount_mrw:
	$(CXX) -std=c++14 -O3 $(INCS) $(SRC_DIR)/run_goalcount_mrw.cc \
  $(SRC_DIR)/goalcount.cc $(SRC_DIR)/sas_data.cc $(SRC_DIR)/trie.cc \
  $(SRC_DIR)/parser.cc -o $(BIN_DIR)/goalcount_mrw

ff_mrw:
	$(CXX) -std=c++14 -O3 $(INCS) $(SRC_DIR)/run_ff_mrw.cc \
  $(SRC_DIR)/graphplan.cc $(SRC_DIR)/sas_data.cc $(SRC_DIR)/trie.cc \
  $(SRC_DIR)/parser.cc -o $(BIN_DIR)/ff_mrw

bfs:
	$(CXX) -std=c++14 -O3 $(INCS) $(SRC_DIR)/bfs.cc $(SRC_DIR)/data.cc \
  $(SRC_DIR)/trie.cc $(SRC_DIR)/parser.cc -o $(BIN_DIR)/bfs

graphplan:
	$(CXX) -std=c++14 -O3 $(INCS) $(SRC_DIR)/run_graphplan.cc \
  $(SRC_DIR)/graphplan.cc $(SRC_DIR)/data.cc $(SRC_DIR)/parser.cc \
  -o $(BIN_DIR)/graphplan

test_data:
	$(CXX) -std=c++14 $(INCS) $(TEST_DIR)/test_data.cc $(TEST_DIR)/data.cc \
  -o $(BIN_DIR)/test_data

test_parser:
	$(CXX) -std=c++14 $(INCS) $(TEST_DIR)/test_parser.cc $(SRC_DIR)/parser.cc \
  $(SRC_DIR)/data.cc -o $(BIN_DIR)/test_parser

test_trie:
	$(CXX) -std=c++14 $(INCS) $(TEST_DIR)/test_trie.cc $(SRC_DIR)/trie.cc \
  $(TEST_DIR)/data.cc -o $(BIN_DIR)/test_trie

clean:
	rm -f $(BIN_DIR)/*
