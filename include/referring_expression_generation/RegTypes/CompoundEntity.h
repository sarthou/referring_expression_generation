#ifndef REG_COMPOUNDENTITY_H
#define REG_COMPOUNDENTITY_H

#include <memory>
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
typedef std::shared_ptr<CompoundEntityLabel> CompoundEntityLabelPtr;

struct labelGraphNode_t
{
  std::string property;
  std::vector<CompoundEntityLabelPtr> possible_labels;
  std::unordered_set<CompoundEntityLabelPtr> valid_labels;
  std::unordered_set<std::string> used_properties;
  std::vector<labelGraphNode_t> next_nodes;

  std::string toString(size_t level = 0)
  {
    std::string res;
    for(size_t i = 0; i < level; i++)
      res += "   ";
    res += property;
    if(valid_labels.size())
      res += " ok " + std::to_string(valid_labels.size());
    res += "\n";
    for(auto& node : next_nodes)
      res += node.toString(level + 1);
    return res;
  }
};

class CompoundEntity
{
public:
  CompoundEntity(const std::string& entity_id, const std::string& class_id);

  void setLabels(const std::vector<std::string>& str_labels);
  bool isUsableProperty(const std::string& property);

  void setSubjectProperty(const std::string& subject_property);
  bool useProperty(const std::string& property);

  std::string toString();
  std::string nodeGraphToString();

  void createLabelsGraph();

  std::string entity_id;
  std::string class_id;
  std::string subject_property;
  std::unordered_set<std::string> possible_properties;
  std::unordered_set<std::string> used_properties;
  std::vector<CompoundEntityLabelPtr> labels;

  std::vector<labelGraphNode_t> labels_nodes;
private:

  std::vector<labelGraphNode_t> createLabelGraphNode(std::vector<CompoundEntityLabelPtr> labels, labelGraphNode_t& previous_node);
};

} // namespace reg

#endif // REG_COMPOUNDENTITY_H
