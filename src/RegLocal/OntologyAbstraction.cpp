#include "referring_expression_generation/RegLocal/OntologyAbstraction.h"

#include <set>
#include <unordered_set>

namespace reg
{

  OntologyAbstraction::OntologyAbstraction(const std::vector<std::string>& onto_files, const std::string& config_path) : interface_(&nh_)
  {
    interface_.setDisplay(false);
    interface_.init("en", "none", onto_files, config_path);
    onto = interface_.getOntology();
    onto_thread_ = std::thread(&ontologenius::RosInterface::run, &interface_);
    interface_.close();
  }

  OntologyAbstraction::~OntologyAbstraction()
  {
    interface_.stop();
    onto_thread_.join();
  }

  std::string OntologyAbstraction::getUsableClass(const std::string& indiv_id)
  {
    size_t depth = 1;
    size_t nb_classes = 0;
    std::set<std::string> explored_classes;

    while(1)
    {
      std::unordered_set<std::string> classes = onto->individual_graph_.getUp(indiv_id, depth);
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

  std::vector<std::string> OntologyAbstraction::getUsableClasses(const std::string& indiv_id)
  {
    size_t depth = 1;
    size_t nb_classes = 0;
    std::set<std::string> explored_classes;
    std::vector<std::string> usableClasses;

    while(1)
    {
      std::unordered_set<std::string> classes = onto->individual_graph_.getUp(indiv_id, depth);
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

    std::string label = onto->individual_graph_.getName(indiv_id, false);
    if(label != "")
    {
      if(onto->individual_graph_.find(label, false).size() == 1)
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
    std::string label = onto->class_graph_.getName(class_id, false);
    if(label != "")
    {
      if(onto->class_graph_.find(label, false).size() == 1)
        return true;
      else
        return false;
    }
    else
      return false;
  }

  std::vector<ontologenius::OntologeniusSparqlResponse> OntologyAbstraction::callSparlql(const std::string& query)
  {
    std::vector<ontologenius::OntologeniusSparqlResponse> res;
    std::vector<std::map<std::string, std::string>> results = interface_.getSparqlInterface()->run(query);

    for(auto& result : results)
    {
      ontologenius::OntologeniusSparqlResponse tmp;
      for(auto& r : result)
      {
        tmp.names.push_back(r.first);
        tmp.values.push_back(r.second);
      }
      res.push_back(tmp);
    }

    return res;
  }

} // namespace reg
