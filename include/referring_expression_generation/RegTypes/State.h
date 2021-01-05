#ifndef REG_STATE_H
#define REG_STATE_H

#include <memory>
#include <set>
#include <unordered_set>

#include "referring_expression_generation/RegTypes/Triplet.h"

namespace reg {

class State;
typedef std::shared_ptr<State> StatePtr;

class State
{
public:
  State(StatePtr ancestor, TripletPtr triplet) : ancestor(ancestor)
  {
    triplets.insert(triplet);
    initial_state = false;
    if(ancestor)
      hash_set_ = ancestor->hash_set_;
    if(triplet)
      hash_set_.insert(triplet->hash_value);
  }

  State(StatePtr ancestor, const std::unordered_set<TripletPtr>& triplets) : ancestor(ancestor)
  {
    initial_state = false;
    this->triplets = std::move(triplets);
    if(ancestor)
      hash_set_ = ancestor->hash_set_;
    if(triplets.size())
    {
      for(auto& triplet : triplets)
        hash_set_.insert(triplet->hash_value);
    }
  }

  StatePtr ancestor;
  std::unordered_set<TripletPtr> triplets;
  bool initial_state;

  bool operator==(const State& other) const
  {
    return (hash_set_ == other.hash_set_);
  }

  bool equals(const StatePtr& other) const
  {
    return (hash_set_ == other->hash_set_);
  }

  std::string toString()
  {
    std::string res;
    for(auto t : triplets)
      res += t->toString() + ", ";
    return res;
  }

private:
  std::set<size_t> hash_set_;
};

} // namespace reg

#endif // REG_STATE_H
