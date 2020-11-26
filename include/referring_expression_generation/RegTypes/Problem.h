#ifndef REG_PROBLEM_H
#define REG_PROBLEM_H

#include <queue>
#include <string>
#include <vector>

#include "referring_expression_generation/RegTypes/Action.h"
#include "referring_expression_generation/RegTypes/Node.h"
#include "referring_expression_generation/RegTypes/State.h"
#include "referring_expression_generation/RegTypes/Triplet.h"

namespace reg {

struct PathCostCompare_t
{
  bool operator()(NodePtr lhs, NodePtr rhs)
  {
    return lhs->path_cost > rhs->path_cost;
  }
};

typedef std::priority_queue<NodePtr, std::vector<NodePtr>, PathCostCompare_t> NodePriorityQueue;

struct IndividualDifferences_t
{
    std::vector<TripletPtr> hard_differences;
    std::vector<TripletPtr> soft_differences;
};
typedef std::shared_ptr<IndividualDifferences_t> IndividualDifferencesPtr;

struct SymbolTable_t
{
  std::vector<std::string> individuals;
  std::vector<std::string> symbols;

  SymbolTable_t(const std::vector<std::string>& individuals = {},
                const std::vector<std::string>& symbols = {}) : individuals(individuals),
                                                                symbols(symbols) {}

  SymbolTable_t(const SymbolTable_t& other) : individuals(other.individuals),
                                              symbols(other.symbols) {}
};

struct Problem_t
{
  std::string goal;
  std::vector<Triplet> context;
  std::vector<std::string> restrictor;
  SymbolTable_t symbols;

  Problem_t(const std::string& goal = "",
            const std::vector<Triplet>& context = {},
            const std::vector<std::string>& restrictor = {},
            SymbolTable_t symbols = SymbolTable_t()) : goal(goal),
                                                       context(context),
                                                       restrictor(restrictor),
                                                       symbols(symbols) {}
};

struct Solution_t
{
  Solution_t(bool success = false) { this->success = success; }

  bool success;
  std::vector<std::string> sparql;
  std::vector<TripletPtr> triplets;
  std::vector<std::string> ambiguous;
};

class SparqlVariables
{
private:
  std::unordered_map<std::string, std::string> indiv_to_variable;
  std::unordered_map<std::string, std::string> variable_to_indiv;
  size_t var_namer;

public:
  SparqlVariables() { var_namer = 0; }

  void set(const std::string& id)
  {
    if(indiv_to_variable.count(id) == 0)
    {
      indiv_to_variable[id] = std::to_string(var_namer++);
      variable_to_indiv[indiv_to_variable[id]] = id;
    }
  }

  void set(const std::string& id, const std::string& num)
  {
    if(indiv_to_variable.count(id) == 0)
    {
      size_t num_id;
      if(sscanf(num.c_str(), "%zu", &num_id) == 1)
      {
        if(num_id > var_namer)
          var_namer = num_id;
      }

      indiv_to_variable[id] = std::to_string(var_namer++);
      variable_to_indiv[indiv_to_variable[id]] = id;
    }
  }

  std::string getIndividual(const std::string& variable)
  {
    return variable_to_indiv[variable];
  }

  std::string getVar(const std::string& id)
  {
    if(indiv_to_variable.count(id) != 0)
      return "?" + indiv_to_variable[id];
    else
      return id;
  }
};


} // namespace reg

#endif // REG_PROBLEM_H
