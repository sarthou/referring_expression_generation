#include "ros/ros.h"

#include "referring_expression_generation/RegServer/Reg.h"
#include "referring_expression_generation/StatsManager.h"

#include "time.h"
#include <chrono>

using namespace std::chrono;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reg_server");
  ros::NodeHandle n;

  reg::Reg solver;
  solver.closeOntology();
  ros::Duration(3).sleep();

  reg::Solution_t solution;
  reg::Problem_t problem("tomato_1", {}, {}); // "VisualRelation"

  StatsManager::getInstance().reset();

  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  clock_t t = clock();

  solution = solver.plan(problem);

  t = clock() - t;
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

  std::cout << "Number of cycles : " << t << std::endl << "Time took : " << ((float)t)/CLOCKS_PER_SEC * 1000 << "ms" << std::endl;
  std::cout << time_span.count()*1000 << "ms" << std::endl;

  if(solution.success)
  {
    std::cout << "SOLVED" << std::endl;
    std::cout << "Node explored : " << StatsManager::getInstance().node_explored  << std::endl;
    std::cout << "SPARQL Requests : " << StatsManager::getInstance().sparql_queries << std::endl;

    for(auto& query_part : solution.sparql)
      std::cout << query_part << ", ";
    std::cout << std::endl;
  }
  else
  {
    std::cout << "fail to solve" << std::endl;
    std::cout << "ambiguous individuals are:" << std::endl;
    for(auto& amb : solution.ambiguous)
      std::cout << "- " << amb << std::endl;
  }

  return 0;
}
