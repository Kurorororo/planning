#include "parser.h"

#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <sstream>
#include <unordered_map>
#include <vector>

#include "sas_data.h"

namespace parser {

const std::string kFileError = "invalid SAS file";
const char kDelimiter = ' ';

int ParseVersion(std::queue<std::string> &lines) {
  if (lines.front() != "begin_version") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
  int version = std::stoi(lines.front());
  lines.pop();
  if (lines.front() != "end_version") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
  return version;
}

int ParseMetric(std::queue<std::string> &lines) {
  if (lines.front() != "begin_metric") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
  int metric = std::stoi(lines.front());
  lines.pop();
  if (lines.front() != "end_metric") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
  return metric;
}

int ParseVariable(std::queue<std::string> &lines) {
  if (lines.front() != "begin_variable") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
  if (lines.front().substr(0, 3) != "var") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
  int n = std::stoi(lines.front());
  lines.pop();
  for (int i=0; i<n; ++i) {
    lines.pop();
  };
  n = std::stoi(lines.front());
  int sup = n;
  lines.pop();
  for (int i=0; i<n; ++i) {
    lines.pop();
  };
  if (lines.front() != "end_variable") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
  return sup;
}

void ParseVariables(std::queue<std::string> &lines,
                    std::vector<int> &variables, std::vector<int> &sups) {
  int n = std::stoi(lines.front());
  lines.pop();
  variables.resize(n);
  sups.resize(n);
  for (int i=0; i<n; ++i)
    sups[i] = ParseVariable(lines);
}

void ParseVarValue(const std::string &line,
                   std::unordered_map<int, int> &var_value) {
  std::string buffer;
  std::istringstream line_separater(line);
  std::getline(line_separater, buffer, kDelimiter);
  int var = std::stoi(buffer);
  std::getline(line_separater, buffer, kDelimiter);
  int value = std::stoi(buffer);
  var_value[var] = value;
}

void ParseMutexGroup(std::queue<std::string> &lines,
                     std::unordered_map<int, int> &mutex_group) {
  if (lines.front() != "begin_mutex_group") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
  int n = std::stoi(lines.front());
  lines.pop();
  for (int i=0; i<n; ++i) {
    ParseVarValue(lines.front(), mutex_group);
    lines.pop();
  }
  if (lines.front() != "end_mutex_group") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
}

void ParseMutexGroups(
    std::queue<std::string> &lines,
    std::vector<std::unordered_map<int, int> > &mutex_groups) {
  int n = std::stoi(lines.front());
  lines.pop();
  mutex_groups.resize(n);
  for (int i=0; i<n; ++i) {
    ParseMutexGroup(lines, mutex_groups[i]);
  }
}

void ParseState(std::queue<std::string> &lines, std::vector<int> &variables) {
  if (lines.front() != "begin_state") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
  for (int i=0, n=variables.size(); i<n; ++i) {
    variables[i] = std::stoi(lines.front());
    lines.pop();
  }
  if (lines.front() != "end_state") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
}

void ParseGoal(std::queue<std::string> &lines,
               std::unordered_map<int, int> &goal) {
  if (lines.front() != "begin_goal") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
  int n = std::stoi(lines.front());
  lines.pop();
  for (int i=0; i<n; ++i) {
    ParseVarValue(lines.front(), goal);
    lines.pop();
  }
  if (lines.front() != "end_goal") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
}

void ParsePrecondition(std::queue<std::string> &lines,
                       std::vector<int> &precondition) {
  std::string buffer;
  int n = std::stoi(lines.front());
  lines.pop();
  for (int i=0; i<n; ++i) {
   std::istringstream line_separater(lines.front());
   std::getline(line_separater, buffer, kDelimiter);
   int var = std::stoi(buffer);
   std::getline(line_separater, buffer, kDelimiter);
   int value = std::stoi(buffer);
   precondition[var] = value;
   lines.pop();
  }
}

void ParseEffects(std::queue<std::string> &lines,
                  std::vector<int> &precondition,
                  std::vector< std::unique_ptr<sas_data::Effect> > &effects) {
  int n = std::stoi(lines.front());
  lines.pop();
  std::string buffer;
  for (int i=0; i<n; ++i) {
    auto effect = sas_data::Effect::Create();
    std::istringstream line_separater(lines.front());
    std::getline(line_separater, buffer, kDelimiter);
    for (int j=0, m=std::stoi(buffer); j<m; ++j) {
      std::getline(line_separater, buffer, kDelimiter);
      int var = std::stoi(buffer);
      std::getline(line_separater, buffer, kDelimiter);
      int value = std::stoi(buffer);
      effect->push_condition(var, value);
    }
    std::getline(line_separater, buffer, kDelimiter);
    int var = std::stoi(buffer);
    std::getline(line_separater, buffer, kDelimiter);
    int value = std::stoi(buffer);
    if (value != -1)
      precondition[var] = value;
    std::getline(line_separater, buffer, kDelimiter);
    value = std::stoi(buffer);
    effect->set_effect(var, value);
    effects.push_back(std::move(effect));
    lines.pop();
  }
}

void ParseOperator(std::queue<std::string> &lines, int metric,
                   std::vector<int> &precondition,
                   std::unique_ptr<sas_data::SASOperator> &sas_operator) {
  if (lines.front() != "begin_operator") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
  sas_operator = sas_data::SASOperator::Create();
  sas_operator->set_name(lines.front());
  lines.pop();
  ParsePrecondition(lines, precondition);
  ParseEffects(lines, precondition, sas_operator->get_effcts());
  if (metric == 0)
    sas_operator->set_cost(1);
  else
    sas_operator->set_cost(std::stoi(lines.front()));
  lines.pop();
  if (lines.front() != "end_operator") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
}

void ParseOperators(
    std::queue<std::string> &lines, int metric,
    int n_variables,
    std::vector< std::vector<int> > &preconditions,
    std::vector< std::unique_ptr<sas_data::SASOperator> > &sas_operators) {
  int n = std::stoi(lines.front());
  preconditions.resize(n);
  sas_operators.resize(n);
  lines.pop();
  for (int i=0; i<n; ++i) {
    preconditions[i].resize(n_variables);
    std::fill_n(preconditions[i].begin(), n_variables, -1);
    ParseOperator(lines, metric, preconditions[i], sas_operators[i]);
  }
}

void Parse(
    const std::string &filename, std::vector<int> &variables,
    std::vector<int> &sups,
    std::vector< std::unordered_map<int, int> > &mutex_groups,
    std::unordered_map<int, int> &goal,
    std::vector< std::vector<int> > &preconditions,
    std::vector< std::unique_ptr<sas_data::SASOperator> > &sas_operators) {
  std::ifstream input;
  input.open(filename, std::ios::in);
  std::string buffer;
  std::queue<std::string> lines;
  while (std::getline(input, buffer)) {
    lines.push(buffer);
  }
  input.close();
  int version = ParseVersion(lines);
  if (version != 3) {
    std::cerr << "SAS version must be 3" << std::endl;
    exit(1);
  }
  int metric = ParseMetric(lines);
  ParseVariables(lines, variables, sups);
  ParseMutexGroups(lines, mutex_groups);
  ParseState(lines, variables);
  ParseGoal(lines, goal);
  ParseOperators(lines, metric, variables.size(), preconditions,
                 sas_operators);
  if (std::stoi(lines.front()) != 0) {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
}

} // namespace parser
