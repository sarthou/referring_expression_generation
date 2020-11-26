#ifndef REG_ACTION_H
#define REG_ACTION_H

#include <vector>

#include "referring_expression_generation/RegTypes/Triplet.h"

namespace reg {

class Action
{
public:
  Action(const std::vector<Triplet>& triplets, int path_cost)
  {
    this->triplets = triplets;
    this->path_cost = path_cost;
  }

  std::vector<Triplet> triplets;
  int path_cost;

  bool operator==(const Action& other) const
  {
    if(triplets.size() != other.triplets.size())
      return false;

    for(auto& triplet : triplets)
    {
      bool found = false;
      for(auto& other_triplet : other.triplets)
        if(triplet.equals(other_triplet))
        {
          found = true;
          break;
        }
      if(found == false)
        return false;
    }
    return true;
  }

  std::string toString()
  {
    std::string res;
    for(auto& triplet : triplets)
      res += triplet.toString() + ", ";
    res += std::to_string(path_cost);
    return res;
  }
};

} // namespace reg

#endif // REG_ACTION_H
