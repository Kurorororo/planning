#ifndef PARSER_H_
#define PARSER_H_

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "sas_data.h"

namespace parser {

void Parse(
    const std::string &filename, std::vector<int> &variables,
    std::vector<int> &sups,
    std::vector< std::unordered_map<int, int> > &mutex_groups,
    std::unordered_map<int, int> &goal,
    std::vector< std::map<int, int> > &preconditions,
    std::vector< std::unique_ptr<sas_data::SASOperator> > &sas_operators);

} // namespace parser

#endif
