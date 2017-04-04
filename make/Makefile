SRC_DIR=../src
INC_DIR=../include
BIN_DIR=../bin

INCS = -I$(INC_DIR) -I/usr/local/include/

UNAME := $(shell uname -s)
ifeq ($(UNAME),Linux)
  CXX=g++
endif
ifeq ($(UNAME),Darwin)
  CXX=/usr/bin/clang++
endif

goal_count:
	$(CXX) -std=c++14 $(INCS) $(SRC_DIR)/run_goal_count.cc \
  $(SRC_DIR)/goal_count.cc $(SRC_DIR)/sas_data.cc $(SRC_DIR)/trie.cc \
  $(SRC_DIR)/parser.cc -O2 -o $(BIN_DIR)/goal_count

bfs:
	$(CXX) -std=c++14 $(INCS) $(SRC_DIR)/bfs.cc $(SRC_DIR)/sas_data.cc \
  $(SRC_DIR)/trie.cc $(SRC_DIR)/parser.cc -O2 -o $(BIN_DIR)/bfs

clean:
	rm -f $(BIN_DIR)/*
