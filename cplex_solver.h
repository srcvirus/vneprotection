#ifndef CPLEX_SOLVER_
#define CPLEX_SOLVER_

#include "datastructure.h"
#include "util.h"

#include <ilcplex/ilocplex.h>

// Type definitions for holding upto 5-dimensional decision variables.
typedef IloArray<IloIntVarArray> IloIntVar2dArray;
typedef IloArray<IloIntVar2dArray> IloIntVar3dArray;
typedef IloArray<IloIntVar3dArray> IloIntVar4dArray;
typedef IloArray<IloIntVar4dArray> IloIntVar5dArray;

typedef IloArray<IloIntArray> IloInt2dArray;
typedef IloArray<IloInt2dArray> IloInt3dArray;

typedef IloArray<IloExprArray> IloExpr2dArray;
using std::string;

void PrintIloInt2dArray(IloInt2dArray &a, int dimension1, int dimension2,
                        string name);

void PrintIloInt3dArray(IloInt3dArray &a, int dimension1, int dimension2,
                        int dimension3, string name);

class VNEProtectionCPLEXSolver {
 public:
  VNEProtectionCPLEXSolver() {}
  VNEProtectionCPLEXSolver(Graph *physical_topology, Graph *virt_topology,
                           Graph *shadow_virt_topology,
                           std::vector<std::vector<int>> *location_constraint);

  IloEnv &env() { return env_; }
  IloModel &model() { return model_; }
  IloCplex &cplex() { return cplex_; }
  IloConstraintArray &constraints() { return constraints_; }
  IloNumArray &preferences() { return preferences_; }
  IloIntVar4dArray &x_mn_uv() { return x_mn_uv_; }
  IloIntVar2dArray &y_m_u() { return y_m_u_; }
  IloExpr &objective() { return objective_; }
  void BuildModel();
  bool Solve();

 private:

  IloEnv env_;
  IloModel model_;
  IloCplex cplex_;
  IloConstraintArray constraints_;
  IloNumArray preferences_;
  Graph *physical_topology_;
  Graph *virt_topology_;
  Graph *shadow_virt_topology_;
  std::vector<std::vector<int>> *location_constraint_;
  // Decision variable for edge mapping.
  IloIntVar4dArray x_mn_uv_;
  // Decision variable for node mapping.
  IloIntVar2dArray y_m_u_;

  // Derived variable denoting if a virtual link originating at m is mapped to a
  // physical path going through a physical node u.
  // (deprecated)
  // IloIntVar2dArray eta_m_u_;

  // Variable indicating location constraint.
  IloInt2dArray l_m_u_;
  // Objective function.
  IloExpr objective_;
};
#endif  // CPLEX_SOLVER_
