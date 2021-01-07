#include "ros/ros.h"

#include "referring_expression_generation/RegLocal/Reg.h"
#include "referring_expression_generation/StatsManager.h"

#include "time.h"
#include <chrono>

using namespace std::chrono;

#define NB_OF_SAMPLES 1000.0

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reg_server");
  ros::NodeHandle n;

  reg::Reg solver({
    "/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/referring_expression_generation/resources/ontology/expe_ontology.owl",
    "/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/referring_expression_generation/resources/ontology/bedroom.owl",
    "/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/referring_expression_generation/resources/ontology/living_room.owl",
    "/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/referring_expression_generation/resources/ontology/office.owl",
    "/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/referring_expression_generation/resources/ontology/performance.owl",
    "/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/referring_expression_generation/resources/ontology/actions.owl"
    },
    "/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/referring_expression_generation/resources/conf/no_generalization.yaml");
  solver.closeOntology();
  ros::Duration(3).sleep();

  reg::Solution_t solution;
  reg::Problem_t problem("knife_2", {}, {}); //"VisualRelation"

  StatsManager::getInstance().reset();

  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  clock_t t = clock();

  for (size_t i=0; i < NB_OF_SAMPLES; i++){
    solution = solver.plan(problem);
  }

  t = clock() - t;
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

  std::cout << "Number of cycles : " << t/NB_OF_SAMPLES << std::endl << "Time took : " << ((float)t)/CLOCKS_PER_SEC/NB_OF_SAMPLES * 1000 << "ms" << std::endl;
  std::cout << time_span.count()/NB_OF_SAMPLES*1000 << "ms" << std::endl;

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
