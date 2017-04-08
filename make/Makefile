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

goalcount:
	$(CXX) -std=c++14 $(INCS) $(SRC_DIR)/run_goalcount.cc \
  $(SRC_DIR)/goalcount.cc $(SRC_DIR)/sas_data.cc $(SRC_DIR)/trie.cc \
  $(SRC_DIR)/parser.cc -o $(BIN_DIR)/goalcount

bfs:
	$(CXX) -std=c++14 $(INCS) $(SRC_DIR)/bfs.cc $(SRC_DIR)/sas_data.cc \
  $(SRC_DIR)/trie.cc $(SRC_DIR)/parser.cc -O2 -o $(BIN_DIR)/bfs

clean:
	rm -f $(BIN_DIR)/*
