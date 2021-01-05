#include "referring_expression_generation/RegServer/Reg.h"
#include "referring_expression_generation/StatsManager.h"

#define DEBUG

namespace reg
{

Reg::Reg(const std::string& onto_name) : onto_(onto_name)
{
}

Solution_t Reg::plan(const Problem_t& problem)
{
  Solution_t solution;
  problem_ = problem;

  explored_.clear();
  frontier_ = NodePriorityQueue();
  variables_ = SparqlVariables();

  usable_properties_.clear();
  unusable_properties_.clear();

  if(onto_.isUsableIndividual(problem_.goal) == false)
  {
    variables_.set(problem_.goal);
    for(size_t i = 0; i < problem.symbols.individuals.size(); i++)
      variables_.set(problem.symbols.individuals[i], problem.symbols.symbols[i]);
    NodePtr init_node = createInitialState(problem_.goal, problem_.context);
    solution = solve(init_node);

    if(solution.sparql.size())
      solution.sparql = std::vector<std::string>(solution.sparql.begin() + init_node->query.size(), solution.sparql.end());
  }

  return solution;
}

Solution_t Reg::replan()
{
  Solution_t solution;

  if(onto_.isUsableIndividual(problem_.goal) == false)
  {
    NodePtr init_node = createInitialState(problem_.goal, problem_.context);
    solution = solve();

    if(solution.sparql.size())
      solution.sparql = std::vector<std::string>(solution.sparql.begin() + init_node->query.size(), solution.sparql.end());
  }

  return solution;
}

Solution_t Reg::solve(NodePtr node)
{
  frontier_.push(node);

  return solve();
}

Solution_t Reg::solve()
{
  NodePtr node = nullptr;
  NodePtr less_ambiguous_node = node;
  int less_ambiguity = 1000;
  bool found = false;

  while(found == false)
  {
    if(frontier_.empty())
      break;

    node = frontier_.top();
    frontier_.pop();
    std::cout << "============= POP ============" << std::endl;
    std::cout << node->state->toString() << std::endl;
    StatsManager::getInstance().node_explored++;

    if(isGoalNode(node))
    {
      found = true;
    }
    else
    {
      if(node->ambiguous[problem_.goal].size() < less_ambiguity)
        less_ambiguous_node = node;
    }

    explored_.push_back(node->state);

    auto actions = getActions(node);
    for(auto& action : actions)
    {
      NodePtr child = getChildNode(node, action);
      if(child != nullptr)
      {
        if(!existInExplored(child->state) && !existInFrontier(child->state))
          frontier_.push(child);
      }
    }
  }

  if(found)
    return getSolution(node);
  else
    return getFailedSolution(less_ambiguous_node);
}

NodePtr Reg::createInitialState(const std::string& individual, const std::vector<Triplet>& base_triplets)
{
  std::string individual_variable = variables_.getVar(individual);

  std::unordered_set<TripletPtr> init_triplets;
  std::vector<std::string> query;
  for(auto t : base_triplets)
  {
    TripletPtr triplet = std::make_shared<Triplet>(
      (t.from == individual_variable) ? individual : t.from,
      t.relation,
      (t.on == individual_variable) ? individual : t.on
    );
    init_triplets.insert(triplet);
    query.push_back(toQuery(triplet));
  }

  NodePtr init_node = std::make_shared<Node>(nullptr, init_triplets);
  init_node->path_cost = 0;
  init_node->state->initial_state = true;

  if(!onto_.isUsableIndividual(individual) && !isAExist(init_node, individual))
      init_node->unnamed_individuals.insert(individual);

  return init_node;
}

bool Reg::isGoalNode(const NodePtr& node)
{
  return ((node->unnamed_individuals.size() == 0) && (getH(node) == 0) && isCompoundEntitiesValid(node));
}

Solution_t Reg::getSolution(NodePtr node)
{
  Solution_t solution(true);

  if(node != nullptr)
    solution.sparql = node->query;

  StatePtr state = node->state;
  while(state != nullptr)
  {
    if(state->initial_state == false)
      for(auto& triplet : state->triplets)
        solution.triplets.push_back(triplet);
    state = state->ancestor;
  }

  return solution;
}

Solution_t Reg::getFailedSolution(NodePtr node)
{
  Solution_t solution(false);

  if(node)
  {
    for(auto& amb : node->ambiguous[problem_.goal])
      solution.ambiguous.push_back(amb);
  }

  return solution;
}

bool Reg::getNamingActions(NodePtr node, std::vector<Action>& actions)
{
  if(node->unnamed_individuals.size() == 0)
    return false;
  else if(node->unnamed_individuals.size() == 1)
  {
    auto unnamed_individual = *(node->unnamed_individuals.begin());
    std::vector<std::string> usableClasses = onto_.getUsableClasses(unnamed_individual);
    for (auto& c: usableClasses)
    {
      variables_.set(unnamed_individual);
      Action action({std::make_shared<Triplet>(unnamed_individual, "isA", c)}, 1);

      if(isCompoundEntity(c))
        action.compound_entities.push_back(std::make_pair(unnamed_individual, c));

      if(!existInAction(action, actions))
      {
        actions.push_back(std::move(action));
        StatsManager::getInstance().undirect_naming_relation_created++;
      }
    }
    return true;
  }
  else
  {
    std::vector<Action> local_actions;
    for(auto& unnamed_individual : node->unnamed_individuals)
    {
      std::vector<std::string> usable_classes = onto_.getUsableClasses(unnamed_individual);
      if(usable_classes.size())
        variables_.set(unnamed_individual);

      if(local_actions.size() == 0)
      {
        for(auto& usable_classe: usable_classes)
        {
          local_actions.push_back(Action({std::make_shared<Triplet>(unnamed_individual, "isA", usable_classe)}, 1));
        }
      }
      else
      {
        std::vector<Action> tmp_actions;
        std::swap(tmp_actions, local_actions);
        bool is_compound_entity = false;
        for(auto& usable_classe: usable_classes)
        {
          is_compound_entity = isCompoundEntity(usable_classe);

          for(auto& action : tmp_actions)
          {
            local_actions.push_back(action);
            local_actions.back().triplets.insert(std::make_shared<Triplet>(unnamed_individual, "isA", usable_classe));
            local_actions.back().path_cost += 1;
            if(is_compound_entity)
              local_actions.back().compound_entities.push_back(std::make_pair(unnamed_individual, usable_classe));
          }
        }
      }
    }

    for(auto& action : local_actions)
    {
      if(!existInAction(action, actions))
      {
        actions.push_back(action);
        StatsManager::getInstance().undirect_naming_relation_created++;
      }
    }

    return true;
  }

  return true;
}

void Reg::getDiffActions(NodePtr node, std::vector<Action>& actions)
{
  for (const auto& kv: node->ambiguous)
  {
    for (auto& amb: kv.second)
    {
      IndividualDifferencesPtr diffs = getIndividualsDifferences(kv.first, amb);
      for (auto& d: diffs->hard_differences)
      {
        if(existInNode(node, d) == false)
        {
          Action action({d}, 1);
          if(isUsableProperty(d->relation))
            if(!existInAction(action, actions))
            {
              actions.push_back(action);
              StatsManager::getInstance().simple_relation_created++;
            }
        }
      }

      for (auto& d: diffs->soft_differences)
      {
        if(existInNode(node, d) == false)
        {
          Action action({d}, 2); // extra cost
          if(isUsableProperty(d->relation))
            if(!existInAction(action, actions))
            {
              actions.push_back(action);
              StatsManager::getInstance().simple_relation_created++;
            }
        }
      }
    }
  }
}

void Reg::getCompoundActions(NodePtr node, std::vector<Action>& actions)
{
  for(auto& compound_entity : node->compound_entities)
  {
    auto properties = compound_entity.second.getNextProperties();
    for(auto& property : properties)
    {
      auto ons = onto_.individuals.getOn(compound_entity.first, property);
      for(auto& on : ons)
      {
        auto d = std::make_shared<Triplet>(compound_entity.first, property, on);
        if(existInNode(node, d) == false)
        {
          Action action({d}, 1); // TODO cost
          if(isUsableProperty(d->relation))
            if(!existInAction(action, actions))
            {
              actions.push_back(action);
              StatsManager::getInstance().compound_relation_created++;
            }
        }
      }
    }
  }
}

std::vector<Action> Reg::getActions(NodePtr node)
{
#ifdef DEBUG
  std::cout << "************* getActions *************" << std::endl;
  std::cout << "unnamed: " << std::endl;
  for(auto& amb : node->unnamed_individuals)
    std::cout << amb << " " << std::endl;
#endif

  std::vector<Action> actions;

  if(getNamingActions(node, actions) == true)
    return actions;

  if(node->ambiguous.size() == 0)
  {
    auto sparql_result = onto_.callSparlql(toQuery(node->query));
    node->ambiguous = getAmbiguous(std::move(sparql_result));
    StatsManager::getInstance().sparql_queries++;
  }

  getDiffActions(node, actions);
  getCompoundActions(node, actions);

#ifdef DEBUG
  std::cout << "found " << actions.size() << " actions" << std::endl;
#endif

  return actions;
}

NodePtr Reg::getChildNode(NodePtr node, Action& action)
{
  NodePtr child = std::make_shared<Node>(node->state, action.triplets);

  child->query = node->query;
  child->compound_entities = node->compound_entities;
  child->unnamed_individuals.clear();

#ifdef DEBUG
  std::cout << "************* create child : " << action.toString() << " *************" << std::endl;
#endif

  std::cout << "nb CE in action ==> " << action.compound_entities.size() << std::endl;
  if(action.compound_entities.size())
  {
    for(auto& compound_entity : action.compound_entities)
    {
      child->compound_entities.insert({compound_entity.first, {compound_entity.first, compound_entity.second}});
      child->compound_entities.at(compound_entity.first).setLabels(onto_.classes.getNames(compound_entity.second));

      auto current_state = child->state;
      while(current_state != nullptr)
      {
        for(auto& triplet : current_state->triplets)
        {
          if(triplet->on == compound_entity.first)
          {
            std::vector<std::string> inv_properties = onto_.individuals.getWith(triplet->on, triplet->from);
            for(auto& inv : inv_properties)
              if(child->compound_entities.at(compound_entity.first).isUsableProperty(inv))
              {
                child->compound_entities.at(compound_entity.first).setSubjectProperty(inv);
                break;
              }
          }
        }
        current_state = current_state->ancestor;
      }

      std::cout << "==> createLabelsGraph" << std::endl;
      child->compound_entities.at(compound_entity.first).createLabelsGraph();
      #ifdef DEBUG
        std::cout << child->compound_entities.at(compound_entity.first).toString() << std::endl;
        std::cout << child->compound_entities.at(compound_entity.first).nodeGraphToString() << std::endl;
      #endif
    }
  }

  for(auto& triplet : action.triplets)
  {
    if(triplet->relation != "isA" && !onto_.isUsableIndividual(triplet->on) && !isAExist(child, triplet->on))
    {
      child->unnamed_individuals.insert(triplet->on);
      variables_.set(triplet->on);
    }

    auto compound_entity_it = child->compound_entities.find(triplet->from);
    if(compound_entity_it != child->compound_entities.end())
    {
      if(compound_entity_it->second.isInvolvedProperty(triplet->relation))
      {
        std::cout << "try to use in CE" << std::endl;
        if(compound_entity_it->second.useProperty(triplet->relation) == false)
          return nullptr;
        #ifdef DEBUG
          std::cout << "on compound_entity " << triplet->from << " : " << triplet->relation << std::endl;
          std::cout << compound_entity_it->second.toString() << std::endl;
          std::cout << compound_entity_it->second.nodeGraphToString() << std::endl;
        #endif
      }
    }

    child->query.push_back(toQuery(triplet));
  }

  child->path_cost = node->path_cost + action.path_cost;

  return child;
}

bool Reg::isAExist(NodePtr node, const std::string& indiv)
{
  if(node->isA_done.find(indiv) != node->isA_done.end())
    return true;

  StatePtr state = node->state;
  while(state != nullptr)
  {
    for(auto& triplet : state->triplets)
    {
      if((triplet->from == indiv) &&
        (triplet->relation == "isA"))
        {
          node->isA_done.insert(indiv);
          return true;
        }
    }

    state = state->ancestor;
  }
  return false;
}

bool Reg::isCompoundEntity(const std::string& class_name)
{
  auto class_names = onto_.classes.getNames(class_name);
  for(auto& name : class_names)
    if(name.find("{?") != std::string::npos)
      return true;
  return false;
}

bool Reg::isUsableProperty(const std::string& property)
{
  if(problem_.restrictor.size() == 0)
    return true;

  if(usable_properties_.find(property) != usable_properties_.end())
    return true;
  if(unusable_properties_.find(property) != unusable_properties_.end())
    return false;

  std::vector<std::string> up_properties = onto_.objectProperties.getUp(property);
  if(up_properties.size() == 0)
     up_properties = onto_.dataProperties.getUp(property);

  for(auto& restiction : problem_.restrictor)
    if(std::find(up_properties.begin(), up_properties.end(), restiction) != up_properties.end())
    {
      usable_properties_.insert(property);
      return true;
    }

  unusable_properties_.insert(property);
  return false;
}

std::unordered_map<std::string, std::vector<std::string>> Reg::getAmbiguous(const std::vector<ontologenius::OntologeniusSparqlResponse>& solutions)
{
  std::unordered_map<std::string, std::vector<std::string>> ambiguous_individuals;
  for (auto& solution: solutions)
  {
    for (size_t i = 0; i < solution.names.size(); i++)
    {
      std::string indiv = variables_.getIndividual(solution.names[i]);
      if (indiv != solution.values[i])
      {
        if (ambiguous_individuals.count(indiv) == 0)
          ambiguous_individuals[indiv] = std::vector<std::string>({solution.values[i]});
        else
          ambiguous_individuals[indiv].push_back(solution.values[i]);
      }
    }
  }
  return ambiguous_individuals;
}

IndividualDifferencesPtr Reg::getIndividualsDifferences(const std::string& indivA, const std::string& indivB)
{
    IndividualDifferencesPtr differences = std::make_shared<IndividualDifferences_t>();
    std::vector<std::string> rel1 = onto_.individuals.getRelationFrom(indivA, 0);
    std::vector<std::string> rel2 = onto_.individuals.getRelationFrom(indivB, 0);
    for (auto& rel: rel1)
    {
      std::vector<std::string> ons = onto_.individuals.getOn(indivA, rel);
      if (std::find(rel2.begin(), rel2.end(), rel) == rel2.end())
      {
        for (auto& on: ons) // If the relation is not present at all in the second individual, put all the triplet with this relation as differences
          differences->soft_differences.push_back(std::make_shared<Triplet>(indivA, rel, on));
      }
      else
      {
        // If the relation is present in the second individual, check if they are pointing the same things
        std::vector<std::string> ons2 = onto_.individuals.getOn(indivB, rel);
        for (auto& on: ons)
        {
          if(std::find(ons2.begin(), ons2.end(), on) == ons2.end())
            differences->hard_differences.push_back(std::make_shared<Triplet>(indivA, rel, on));
        }
      }
    }

    return differences;
}

int Reg::getH(const NodePtr& node)
{
#ifdef DEBUG
  std::cout << "----- getH -----" << std::endl;
#endif
  if(node->ambiguous.size() == 0)
  {
    auto sparql_result = onto_.callSparlql(toQuery(node->query));
    node->ambiguous = getAmbiguous(std::move(sparql_result));
    StatsManager::getInstance().sparql_queries++;
  }

  return node->ambiguous[problem_.goal].size();
}

bool Reg::isCompoundEntitiesValid(const NodePtr& node)
{
  for(auto& compound_entity : node->compound_entities)
    if(compound_entity.second.isValid() == false)
      return false;
  return true;
}

std::string Reg::toQuery(const std::vector<std::string>& sub_queries)
{
  std::string query;
  for(auto sub_query : sub_queries)
  {
    if(query != "")
      query += ", ";
    query += sub_query;
  }
  return query;
}

std::string Reg::toQuery(const TripletPtr& triplet)
{
  return variables_.getVar(triplet->from) + " " +
                          triplet->relation + " " +
                          variables_.getVar(triplet->on);
}

bool Reg::existInNode(NodePtr node, const TripletPtr& triplet)
{
  if(triplet->on.find('#') == std::string::npos)
  {
    std::vector<std::string> inverse = onto_.objectProperties.getInverse(triplet->relation);
    if(inverse.size())
    {
      std::vector<TripletPtr> triplets;
      triplets.push_back(triplet);

      for(const auto& i: inverse)
        triplets.push_back(std::make_shared<Triplet>(triplet->on, i, triplet->from));

      return testExistInNode(node, triplets);
    }
    else
      return testExistInNode(node, triplet);
  }
  else
    return testExistInNode(node, triplet);
}

bool Reg::testExistInNode(NodePtr node, const std::vector<TripletPtr>& triplets)
{
  StatePtr state = node->state;
  while(state != nullptr)
  {
    for(auto& node_triplet : state->triplets)
    {
      for(auto& triplet : triplets)
        if(triplet->equals(node_triplet))
          return true;
    }
    state = state->ancestor;
  }

  return false;
}

bool Reg::testExistInNode(NodePtr node, const TripletPtr& triplet)
{
  StatePtr state = node->state;
  while(state != nullptr)
  {
    for(auto& node_triplet : state->triplets)
    {
      if(triplet->equals(node_triplet))
        return true;
    }
    state = state->ancestor;
  }

  return false;
}

bool Reg::existInAction(const Action& action, const std::vector<Action>& actions)
{
  return (std::find(actions.begin(), actions.end(), action) != actions.end());
}

bool Reg::existInExplored(StatePtr state)
{
  for(auto& explored_state : explored_)
    if(state->equals(explored_state))
      return true;
  return false;
}

bool Reg::existInFrontier(StatePtr state)
{
  NodePriorityQueue tmp = frontier_;

  while(!tmp.empty())
  {
    if(state->equals(tmp.top()->state))
      return true;
    tmp.pop();
  }
  return false;
}

} // namespace reg
