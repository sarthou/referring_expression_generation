#ifndef REG_ACTION_H
#define REG_ACTION_H

#include <unordered_set>

#include "referring_expression_generation/RegTypes/Triplet.h"

namespace reg {

class Action
{
public:
  Action(const std::unordered_set<TripletPtr>& triplets, int path_cost)
  {
    this->triplets = std::move(triplets);
    this->path_cost = path_cost;
  }

  std::unordered_set<TripletPtr> triplets;
  std::vector<std::pair<std::string, std::string>> compound_entities;
  int path_cost;

  bool operator==(const Action& other) const
  {
    if(triplets.size() != other.triplets.size())
      return false;
    else if(triplets.size() == 1)
      return (*triplets.begin())->equals(*other.triplets.begin());
    else
    {
      for(auto& triplet : triplets)
      {
        bool found = false;
        for(auto& other_triplet : other.triplets)
          if(triplet->equals(other_triplet))
          {
            found = true;
            break;
          }
        if(found == false)
          return false;
      }
      return true;
    }
  }

  std::string toString()
  {
    std::string res;
    for(auto& triplet : triplets)
      res += triplet->toString() + ", ";
    res += std::to_string(path_cost);
    return res;
  }
};

} // namespace reg

#endif // REG_ACTION_H
