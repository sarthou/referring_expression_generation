#include "referring_expression_generation/RegTypes/CompoundEntity.h"

#include <map>
#include <iostream>

namespace reg {

CompoundEntityLabel::CompoundEntityLabel(const std::string& label)
{
  this->label = std::move(label);
  size_t pose = this->label.find("{");
  while(pose != std::string::npos)
  {
    pose++;
    size_t end_pose = this->label.find("}", pose);
    std::string property = this->label.substr(pose, end_pose - pose);
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

    pose = this->label.find("{", end_pose);
  }
}

CompoundEntity::CompoundEntity(const std::string& entity_id, const std::string& class_id)
{
  this->entity_id = std::move(entity_id);
  this->class_id = std::move(class_id);
}

void CompoundEntity::setLabels(const std::vector<std::string>& str_labels)
{
  for(auto& str_label : str_labels)
  {
    CompoundEntityLabelPtr label = std::make_shared<CompoundEntityLabel>(str_label);
    if(label->involved_properties.empty() == false)
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

bool CompoundEntity::isUsedProperty(const std::string& property)
{
  return (used_properties.find(property) != used_properties.end());
}

bool CompoundEntity::isInvolvedProperty(const std::string& property)
{
  return (involved_properties.find(property) != involved_properties.end());
}

bool CompoundEntity::hasDirectProperty()
{
  return labels_node->hasDirectNode();
}

std::string CompoundEntity::getDirectProperty()
{
  if(!hasDirectProperty())
    return "";
  else
    return labels_node->next_nodes[0]->property;
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
  involved_properties.insert(subject_property);
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

  bool is_used = false;
  for(auto& node : labels_node->next_nodes)
    if(node->property == property)
    {
      labels_node = node;
      is_used = true;
      break;
    }

  return is_used;
}

std::vector<std::string> CompoundEntity::getNextProperties()
{
  std::vector<std::string> res;
  res.reserve(labels_node->next_nodes.size());
  for(auto& node : labels_node->next_nodes)
    res.push_back(node->property);

  return res;
}

std::string CompoundEntity::toString()
{
  std::string res = "-- compound entity --\n";
  res += entity_id + " : " + class_id + "\n";
  res += "subject = " + subject_property + "\n";
  res += "involved properties = ";
  for(auto& p : involved_properties)
    res += p + ", ";
  res += "\npossible properties = ";
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
  return labels_node->toString();
}

void CompoundEntity::createLabelsGraph()
{
  labels_node = std::make_shared<labelGraphNode_t>(subject_property);
  labels_node->used_properties.insert(subject_property);
  labels_node->next_nodes = createLabelGraphNode(labels, labels_node);
}

std::vector<labelGraphNodePtr> CompoundEntity::createLabelGraphNode(std::vector<CompoundEntityLabelPtr> labels, labelGraphNodePtr previous_node)
{
  std::vector<labelGraphNodePtr> res;

  while(labels.size())
  {
    std::map<std::string, size_t> properties_count;
    std::string max_property = "";
    size_t max_count = 0;

    for(auto& label : labels)
    {
      bool valid_label = true;
      for(auto& property : label->involved_properties)
      {
        if(previous_node->used_properties.find(property) != previous_node->used_properties.end())
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
        previous_node->valid_labels.insert(label);
    }

    if(max_property == "")
      break;

    labelGraphNodePtr label_node = std::make_shared<labelGraphNode_t>(max_property);
    label_node->used_properties = previous_node->used_properties;
    label_node->used_properties.insert(max_property);

    for(auto it = labels.begin(); it != labels.end();)
    {
      if((*it)->involved_properties.find(max_property) != (*it)->involved_properties.end())
      {
        label_node->possible_labels.push_back((*it));
        labels.erase(it);
      }
      else
        ++it;
    }

    label_node->next_nodes = createLabelGraphNode(label_node->possible_labels, label_node);
    res.push_back(label_node);
  }

  return res;
}

} // namespace reg
