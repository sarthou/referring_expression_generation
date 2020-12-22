#include "referring_expression_generation/RegTypes/CompoundEntity.h"

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
    CompoundEntityLabel label(str_label);
    if(label.involved_properties.size())
    {
      for(auto& prop : label.involved_properties)
        possible_properties.insert(prop);
      labels.emplace_back(label);
    }
  }
}

bool CompoundEntity::isUsableProperty(const std::string& property)
{
  return (possible_properties.find(property) != possible_properties.end());
}

void CompoundEntity::setSubjectProperty(const std::string& subject_property)
{
  this->subject_property = subject_property;
  for(auto it = labels.begin(); it != labels.end();)
  {
    if(it->subject_property != subject_property)
      labels.erase(it);
    else
      ++it;
  }

  possible_properties.clear();
  for(auto& label : labels)
    for(auto& prop : label.involved_properties)
      if(prop != subject_property)
        possible_properties.insert(prop);
  used_properties.insert(subject_property);
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
    res += "\t"+ l.label +"\n";

  return res;
}

} // namespace reg
