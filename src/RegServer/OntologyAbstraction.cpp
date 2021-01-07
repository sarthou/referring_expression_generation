#include "referring_expression_generation/RegServer/OntologyAbstraction.h"

#include <set>

//#define DEBUG

namespace reg
{
  std::string OntologyAbstraction::getUsableClass(const std::string& indiv_id)
  {
    size_t depth = 1;
    size_t nb_classes = 0;
    std::set<std::string> explored_classes;

    while(1)
    {
      std::vector<std::string> classes = this->individuals.getUp(indiv_id, depth);
      if(classes.size() == nb_classes)
        return "";
      else
      {
        nb_classes = classes.size();
        for(auto& class_id : classes)
        {
          if (explored_classes.count(class_id) != 0)
              continue;
          explored_classes.insert(class_id);
          if(isUsableClass(class_id))
            return class_id;
        }
      }
      depth++;
    }

    return "";
  }

  std::vector<std::string> OntologyAbstraction::getUsableClasses(const std::string& indiv_id){
    size_t depth = 1;
    size_t nb_classes = 0;
    std::set<std::string> explored_classes;
    std::vector<std::string> usableClasses;

    while(1)
    {
      std::vector<std::string> classes = this->individuals.getUp(indiv_id, depth);
      if(classes.size() == nb_classes)
        return {};
      else
      {
        nb_classes = classes.size();
        for(auto& class_id : classes)
        {
          if (explored_classes.count(class_id) != 0)
              continue;
          explored_classes.insert(class_id);
          if(isUsableClass(class_id))
            usableClasses.push_back(class_id);
        }
        if (usableClasses.size() != 0){
          return usableClasses;
        }
      }
      depth++;
    }

    return {};
  }

  bool OntologyAbstraction::isUsableIndividual(const std::string& indiv_id)
  {
    if(unusable_individuals_.find(indiv_id) != unusable_individuals_.end())
      return false;

    if(indiv_id.find('#') != std::string::npos)
      return true;

    std::string label = this->individuals.getName(indiv_id, false);
    if(label != "")
    {
      if(this->individuals.find(label, false).size() == 1)
        return true;
      else
      {
        unusable_individuals_.insert(indiv_id);
        return false;
      }
    }
    else
    {
      unusable_individuals_.insert(indiv_id);
      return false;
    }
  }

  bool OntologyAbstraction::isUsableClass(const std::string& class_id)
  {
    std::string label = this->classes.getName(class_id, false);
    if(label != "")
    {
      if(this->classes.find(label, false).size() == 1)
        return true;
      else
        return false;
    }
    else
      return false;
  }

  std::vector<ontologenius::OntologeniusSparqlResponse> OntologyAbstraction::callSparlql(const std::string& query)
  {
#ifdef DEBUG
    std::cout << "[OntologyAbstraction] Sparql \"" << query << "\"" << std::endl;
#endif
    return this->sparql.call(query);
  }

} // namespace reg
