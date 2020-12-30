#include "referring_expression_generation/RegTypes/CompoundEntity.h"

#include <map>
#include <iostream>

namespace reg {

CompoundEntityLabel::CompoundEntityLabel(const std::string& label)
{
  this->label = label;
  size_t pose = label.find("{");
  while(pose != std::string::npos)
  {
    pose++;
    size_t end_pose = label.find("}", pose);
    std::string property = label.substr(pose, end_pose - pose);
    if(property.empty() == false)
    {
      if(property[0] == '?')
      {
        property = property.substr(1);
        if(subject_property.empty() == true)
          subject_property = property;
      }
      involved_properties.insert(property);
    }

    pose = label.find("{", end_pose);
  }
}

CompoundEntity::CompoundEntity(const std::string& entity_id, const std::string& class_id)
{
  this->entity_id = entity_id;
  this->class_id = class_id;
}

void CompoundEntity::setLabels(const std::vector<std::string>& str_labels)
{
  for(auto& str_label : str_labels)
  {
    CompoundEntityLabelPtr label = std::make_shared<CompoundEntityLabel>(str_label);
    if(label->involved_properties.size())
    {
      for(auto& prop : label->involved_properties)
        possible_properties.insert(prop);
      labels.emplace_back(label);
    }
  }
}

bool CompoundEntity::isUsableProperty(const std::string& property)
{
  return (possible_properties.find(property) != possible_properties.end());
}

bool CompoundEntity::isInvolvedProperty(const std::string& property)
{
  return (involved_properties.find(property) != involved_properties.end());
}

void CompoundEntity::setSubjectProperty(const std::string& subject_property)
{
  this->subject_property = subject_property;
  for(auto it = labels.begin(); it != labels.end();)
  {
    if((*it)->subject_property != subject_property)
      labels.erase(it);
    else
      ++it;
  }

  possible_properties.clear();
  for(auto& label : labels)
    for(auto& prop : label->involved_properties)
      if(prop != subject_property)
        possible_properties.insert(prop);
  involved_properties = possible_properties;
  used_properties.insert(subject_property);
}

bool CompoundEntity::useProperty(const std::string& property)
{
  auto possible_property_it = possible_properties.find(property);
  if(possible_property_it == possible_properties.end())
    return false;
  possible_properties.erase(possible_property_it);
  used_properties.insert(property);

  for(auto it = labels.begin(); it != labels.end();)
  {
    if((*it)->involved_properties.find(property) == (*it)->involved_properties.end())
      labels.erase(it);
    else
      ++it;
  }

  for(auto& node : labels_node.next_nodes)
    if(node.property == property)
    {
      labels_node = node;
      break;
    }

  return true;
}

std::vector<std::string> CompoundEntity::getNextProperties()
{
  std::vector<std::string> res;
  for(auto& node : labels_node.next_nodes)
    res.push_back(node.property);

  return res;
}

std::string CompoundEntity::toString()
{
  std::string res = "-- compound entity --\n";
  res += entity_id + " : " + class_id + "\n";
  res += "subject = " + subject_property + "\n";
  res += "usable properties = ";
  for(auto& p : possible_properties)
    res += p + ", ";
  res += "\nUsed properties = ";
  for(auto& p : used_properties)
    res += p + ", ";
  res += "\nActive labels =\n";
  for(auto& l : labels)
    res += "\t"+ l->label +"\n";

  return res;
}

std::string CompoundEntity::nodeGraphToString()
{
  return labels_node.toString();
}

void CompoundEntity::createLabelsGraph()
{
  labels_node.property = subject_property;
  labels_node.used_properties.insert(subject_property);
  labels_node.next_nodes = createLabelGraphNode(labels, labels_node);
}

std::vector<labelGraphNode_t> CompoundEntity::createLabelGraphNode(std::vector<CompoundEntityLabelPtr> labels, labelGraphNode_t& previous_node)
{
  std::vector<labelGraphNode_t> res;

  while(labels.size())
  {
    labelGraphNode_t label_node;

    std::map<std::string, size_t> properties_count;
    std::string max_property = "";
    size_t max_count = 0;

    for(auto& label : labels)
    {
      bool valid_label = true;
      for(auto& property : label->involved_properties)
      {
        if(previous_node.used_properties.find(property) != previous_node.used_properties.end())
          continue;

        valid_label = false;
        auto properties_count_it = properties_count.find(property);
        if(properties_count_it == properties_count.end())
        {
          properties_count[property] = 1;
          if(max_count == 0)
          {
            max_count = 1;
            max_property = property;
          }
        }
        else
        {
          properties_count_it->second++;
          if(properties_count_it->second > max_count)
          {
            max_count = properties_count_it->second;
            max_property = property;
          }
        }
      }

      if(valid_label)
        previous_node.valid_labels.insert(label);
    }
    if(max_property == "")
      break;
    label_node.property = max_property;
    label_node.used_properties = previous_node.used_properties;
    label_node.used_properties.insert(max_property);

    for(auto it = labels.begin(); it != labels.end();)
    {
      if((*it)->involved_properties.find(max_property) != (*it)->involved_properties.end())
      {
        label_node.possible_labels.push_back((*it));
        labels.erase(it);
      }
      else
        ++it;
    }

    label_node.next_nodes = createLabelGraphNode(label_node.possible_labels, label_node);
    res.push_back(label_node);
  }

  return res;
}

/*
std::string property;
std::unordered_set<CompoundEntityLabelPtr> possible_labels;
std::unordered_set<CompoundEntityLabelPtr> valid_labels_id;
std::vector<labelGraphNode_t> next_nodes;
*/

} // namespace reg
