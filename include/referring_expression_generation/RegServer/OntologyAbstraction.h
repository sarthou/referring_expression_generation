#ifndef REG_ONTOLOGYABSTRACTION_H
#define REG_ONTOLOGYABSTRACTION_H

#include <string>
#include <vector>

#include <ros/ros.h>

#include "ontologenius/OntologeniusSparqlResponse.h"
#include "ontologenius/OntologiesManipulator.h"

namespace reg
{

class OntologyAbstraction : public OntologyManipulator
{
public:
  OntologyAbstraction(const std::string& name) : OntologyManipulator(name) {}

  std::string getUsableClass(const std::string& indiv_id);
  std::vector<std::string> getUsableClasses(const std::string& indiv_id);

  bool isUsableIndividual(const std::string& indiv_id);
  bool isUsableClass(const std::string& class_id);

  std::vector<ontologenius::OntologeniusSparqlResponse> callSparlql(const std::string& query);

private:
};

} // namespace reg

#endif // REG_ONTOLOGYABSTRACTION_H
