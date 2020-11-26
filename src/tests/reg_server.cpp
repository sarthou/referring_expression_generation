#include "ros/ros.h"

#include "referring_expression_generation/RegServer/Reg.h"
#include "referring_expression_generation/StatsManager.h"

#include "time.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reg_server");
  ros::NodeHandle n;

  reg::Reg solver;
  solver.closeOntology();
  ros::Duration(3).sleep();

  reg::Solution_t solution;
  reg::Problem_t problem("room_B_book_7", {}, {"VisualRelation"});

  StatsManager::getInstance().reset();

  solution = solver.plan(problem);

  if(solution.success)
  {
    std::cout << "SOLVED" << std::endl;
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
