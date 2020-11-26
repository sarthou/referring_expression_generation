#ifndef REG_STATSMANAGER_H
#define REG_STATSMANAGER_H

#include <time.h>

class StatsManager{

public:
  static StatsManager& getInstance(){
      static StatsManager instance;
      return instance;
  }

  void reset(){
      node_explored = 0;
      undirect_naming_relation_created = 0;
      simple_relation_created = 0;
      compound_relation_created = 0;
      sparql_queries = 0;
  }

  unsigned long node_explored;
  unsigned long undirect_naming_relation_created;
  unsigned long simple_relation_created;
  unsigned long compound_relation_created;
  unsigned long sparql_queries;


private:
  StatsManager(){}

public:
  StatsManager(StatsManager const&) = delete;
  void operator=(StatsManager const&) = delete;

};

#endif // REG_STATSMANAGER_H
