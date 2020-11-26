#ifndef REG_TRIPLET_H
#define REG_TRIPLET_H

#include <memory>
#include <string>

namespace reg {

class Triplet;
typedef std::shared_ptr<Triplet> TripletPtr;

class Triplet
{
public:
  std::string from;
  std::string relation;
  std::string on;
  size_t hash_value;

  Triplet(const std::string& from = "", const std::string& relation = "", const std::string& on = ""): from(from), relation(relation), on(on)
  {
    hash_value = std::hash<std::string>{}(from+relation+on);
  }

  inline std::string toString() const
  {
    return std::string(from + " - " + relation + " - " + on);
  }

  inline bool equals(const Triplet& other) const
  {
    return (hash_value == other.hash_value);
  }

  inline bool operator==(const Triplet& other) const
  {
    return (hash_value == other.hash_value);
  }

  inline bool empty() const
  {
    return equals(Triplet{});
  }

  inline bool equals(const TripletPtr& other) const
  {
    return (hash_value == other->hash_value);
  }

  inline bool isInFactsBase(const std::vector<TripletPtr>& factsBase) const
  {
    for (const auto& f: factsBase)
    {
      if (equals(f))
        return true;
    }
    return false;
  }
};

} // namespace reg

#endif // REG_TRIPLET_H
