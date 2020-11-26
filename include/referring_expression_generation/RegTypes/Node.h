#ifndef REG_NODE_H
#define REG_NODE_H

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "referring_expression_generation/RegTypes/State.h"
#include "referring_expression_generation/RegTypes/Triplet.h"

namespace reg {

class Node;
typedef std::shared_ptr<Node> NodePtr;

class Node
{
public:
  Node(StatePtr ancestor, TripletPtr triplet)
  {
    state = std::make_shared<State>(ancestor, triplet);
  }

  Node(StatePtr ancestor, const std::vector<TripletPtr>& triplets)
  {
    state = std::make_shared<State>(ancestor, triplets);
  }

  int path_cost;
  StatePtr state;
  std::vector<std::string> query;
  std::unordered_map<std::string, std::vector<std::string>> ambiguous;
  std::set<std::string> unnamed_individuals;
  std::unordered_set<std::string> isA_done;

  bool operator==(const Node& other) const
  {
    return state->equals(other.state);
  }

  std::string strQuery() const
  {
    std::string res;
    for(auto& i : query)
      res += i + " , ";
    return res;
  }
};

} // namespace reg

#endif // REG_NODE_H
