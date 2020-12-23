#ifndef REG_REGSERVER_H
#define REG_REGSERVER_H

#include <vector>
#include <unordered_map>
#include <string>
#include <unordered_set>

#include "referring_expression_generation/RegServer/OntologyAbstraction.h"
#include "referring_expression_generation/RegTypes/Problem.h"

namespace reg
{

class Reg
{
public:
  Reg(const std::string& onto_name = "");

  void closeOntology() { onto_.close(); }

  Solution_t plan(const Problem_t& problem);
  Solution_t replan();

  std::string toQuery(const std::vector<std::string>& sub_queries);

private:
  OntologyAbstraction onto_;
  NodePriorityQueue frontier_;
  std::vector<StatePtr> explored_;

  std::unordered_set<std::string> usable_properties_;
  std::unordered_set<std::string> unusable_properties_;

  Problem_t problem_;
  SparqlVariables variables_;

  Solution_t solve(NodePtr node);
  Solution_t solve();

  NodePtr createInitialState(const std::string& individual, const std::vector<Triplet>& base_triplets);
  bool isGoalNode(const NodePtr& node);
  Solution_t getSolution(NodePtr node);
  Solution_t getFailedSolution(NodePtr node);

  bool getNamingActions(NodePtr node, std::vector<Action>& actions);
  void getDiffActions(NodePtr node, std::vector<Action>& actions);
  void getCompoundActions(NodePtr node, std::vector<Action>& actions);
  std::vector<Action> getActions(NodePtr node);
  NodePtr getChildNode(NodePtr node, Action& action);

  bool isAExist(NodePtr node, const std::string& indiv);
  bool isCompoundEntity(const std::string& class_name);
  bool isUsableProperty(const std::string& property);

  std::unordered_map<std::string, std::vector<std::string>> getAmbiguous(const std::vector<ontologenius::OntologeniusSparqlResponse>& solutions);
  IndividualDifferencesPtr getIndividualsDifferences(const std::string& indivA, const std::string& indivB);
  int getH(const NodePtr& node);
  bool isCompoundEntitiesValid(const NodePtr& node);

  std::string toQuery(const TripletPtr& triplet);

  bool existInNode(NodePtr node, const TripletPtr& triplet);
  bool testExistInNode(NodePtr node, const std::vector<TripletPtr>& triplets);
  bool testExistInNode(NodePtr node, const TripletPtr& triplet);
  bool existInAction(const Action& action, const std::vector<Action>& actions);

  bool existInExplored(StatePtr state);
  bool existInFrontier(StatePtr state);
};

} // namespace reg

#endif // REG_REGSERVER_H
