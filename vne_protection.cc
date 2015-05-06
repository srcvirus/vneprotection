#include "cplex_solver.h"
#include "datastructure.h"
#include "io.h"
#include "util.h"
#include "vne_solution_builder.h"

#include <iostream>

const std::string kUsage = "./vne_protection "
                           "--pn_topology_file=<pn_topology_file>\n\t"
                           "--vn_topology_file=<vn_topology_file>\n\t"
                           "--location_constraint_file=<location_constraint_file>";

int main(int argc, char *argv[]) {
  using std::string;
  auto arg_map = ParseArgs(argc, argv);
  string pn_topology_filename = "";
  string vn_topology_filename = "";
  string location_constraint_filename = "";
  for (auto argument : *arg_map) {
    if (argument.first == "--pn_topology_file") {
      pn_topology_filename = argument.second;
    } else if (argument.first == "--vn_topology_file") {
      vn_topology_filename = argument.second;
    } else if (argument.first == "--location_constraint_file") {
      location_constraint_filename = argument.second;
    } else {
      printf("Invalid command line option: %s\n", argument.first.c_str());
      return 1;
    }
  }
  auto physical_topology =
      InitializeTopologyFromFile(pn_topology_filename.c_str());
  DEBUG(physical_topology->GetDebugString().c_str());
  auto virt_topology = InitializeTopologyFromFile(vn_topology_filename.c_str());
  DEBUG(virt_topology->GetDebugString().c_str());
  auto shadow_virt_topology =
      InitializeTopologyFromFile(vn_topology_filename.c_str());
  DEBUG(shadow_virt_topology->GetDebugString().c_str());
  auto location_constraints = InitializeVNLocationsFromFile(
      location_constraint_filename.c_str(), virt_topology->node_count());
  auto vne_cplex_solver =
      std::unique_ptr<VNEProtectionCPLEXSolver>(new VNEProtectionCPLEXSolver(
          physical_topology.get(), virt_topology.get(),
          shadow_virt_topology.get(), location_constraints.get()));
  try {
    auto &cplex_env = vne_cplex_solver->env();
    vne_cplex_solver->BuildModel();
    bool is_success = vne_cplex_solver->Solve();
    if (!is_success) {
      auto &cplex = vne_cplex_solver->cplex();
      std::cout << "Solution status: " << cplex.getStatus() << std::endl;
      std::cout << "X : " << cplex.getCplexStatus() << std::endl;
    } else {
      printf("Run successfully completed.\n");
      auto solution_builder = std::unique_ptr<VNESolutionBuilder>(
          new VNESolutionBuilder(vne_cplex_solver.get(),
                                 physical_topology.get(), virt_topology.get()));
      solution_builder->PrintCost((vn_topology_filename + ".cost").c_str());
      solution_builder->PrintWorkingNodeMapping((vn_topology_filename + ".nmap")
                                                    .c_str());
      solution_builder->PrintWorkingEdgeMapping((vn_topology_filename + ".emap")
                                                    .c_str());
      solution_builder->PrintShadowNodeMapping((vn_topology_filename + ".snmap")
                                                   .c_str());
      solution_builder->PrintShadowEdgeMapping((vn_topology_filename + ".semap")
                                                   .c_str());
    }
  }
  catch (IloException & e) {
    printf("Exception thrown: %s\n", e.getMessage());
  }
  return 0;
}
