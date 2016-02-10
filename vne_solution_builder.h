#ifndef VNE_SOLUTION_BUILDER_H_
#define VNE_SOLUTION_BUILDER_H_

#include "cplex_solver.h"

class VNESolutionBuilder {
 public:
  VNESolutionBuilder(VNEProtectionCPLEXSolver *vne_solver_ptr,
                     Graph *physical_topology, Graph *virt_topology)
      : vne_solver_ptr_(vne_solver_ptr),
        physical_topology_(physical_topology),
        virt_topology_(virt_topology) {}

  void PrintWorkingEdgeMapping(const char *filename);
  void PrintShadowEdgeMapping(const char *filename);
  void PrintWorkingNodeMapping(const char *filename);
  void PrintShadowNodeMapping(const char *filename);
  void PrintSolutionStatus(const char *filename);
  void PrintCost(const char *filename);

 private:
  VNEProtectionCPLEXSolver *vne_solver_ptr_;
  Graph *physical_topology_;
  Graph *virt_topology_;
};

#endif  // VNE_SOLUTION_BUILDER_H_
