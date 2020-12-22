#ifndef REG_COMPOUNDENTITY_H
#define REG_COMPOUNDENTITY_H

#include <string>
#include <unordered_set>
#include <vector>

namespace reg {

class CompoundEntityLabel
{
public:
  CompoundEntityLabel(const std::string& label);

  std::string label;
  std::string subject_property;
  std::unordered_set<std::string> involved_properties;
};

class CompoundEntity
{
public:
  CompoundEntity(const std::string& entity_id, const std::string& class_id);

  void setLabels(const std::vector<std::string>& str_labels);
  bool isUsableProperty(const std::string& property);
  void setSubjectProperty(const std::string& subject_property);

  std::string toString();

  std::string entity_id;
  std::string class_id;
  std::string subject_property;
  std::unordered_set<std::string> possible_properties;
  std::unordered_set<std::string> used_properties;
  std::vector<CompoundEntityLabel> labels;
private:
};

} // namespace reg

#endif // REG_COMPOUNDENTITY_H
