#ifndef REG_LOCAL_ONTOLOGYABSTRACTION_H
#define REG_LOCAL_ONTOLOGYABSTRACTION_H

#include <string>
#include <vector>
#include <unordered_set>
#include <thread>

#include <ros/ros.h>

#include "ontologenius/OntologiesManipulator.h"
#include "ontologenius/OntologeniusSparqlResponse.h"

#include "ontologenius/RosInterface.h"

namespace reg
{

class OntologyAbstraction
{
public:
  OntologyAbstraction(const std::vector<std::string>& onto_files, const std::string& config_path);
  ~OntologyAbstraction();

  ontologenius::Ontology* onto;

  void close() { interface_.close(); }

  std::string getUsableClass(const std::string& indiv_id);
  std::vector<std::string> getUsableClasses(const std::string& indiv_id);

  void resetUnusableIndividuals() { unusable_individuals_.clear(); }
  bool isUsableIndividual(const std::string& indiv_id);
  bool isUsableClass(const std::string& class_id);

  std::vector<ontologenius::OntologeniusSparqlResponse> callSparlql(const std::string& query);

private:
  ros::NodeHandle nh_;
  ontologenius::RosInterface interface_;
  std::thread onto_thread_;

  std::unordered_set<std::string> unusable_individuals_;

  std::string name_;
};

} // namespace reg

#endif // REG_LOCAL_ONTOLOGYABSTRACTION_H
