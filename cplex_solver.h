#ifndef CPLEX_SOLVER_
#define CPLEX_SOLVER_

#include "datastructure.h"
#include "util.h"

#include <ilcplex/ilocplex.h>
ILOSTLBEGIN;

// Type definitions for holding upto 5-dimensional decision variables.
typedef IloArray<IloIntVarArray> IloIntVar2dArray;
typedef IloArray<IloIntVar2dArray> IloIntVar3dArray;
typedef IloArray<IloIntVar3dArray> IloIntVar4dArray;
typedef IloArray<IloIntVar4dArray> IloIntVar5dArray;

typedef IloArray<IloIntArray> IloInt2dArray;
typedef IloArray<IloInt2dArray> IloInt3dArray;

typedef IloArray<IloExprArray> IloExpr2dArray;

void PrintIloInt2dArray(IloInt2dArray &a, int dimension1, int dimension2,
                        string name) {
  DEBUG("%s\n", name.c_str());
  for (int i = 0; i < dimension1; ++i) {
    for (int j = 0; j < dimension2; ++j) {
      DEBUG("%d ", a[i][j]);
    }
    DEBUG("\n");
  }
}

void PrintIloInt3dArray(IloInt3dArray &a, int dimension1, int dimension2,
                        int dimension3, string name) {
  DEBUG("%s\n", name.c_str());
  for (int i = 0; i < dimension1; ++i) {
    DEBUG("dim1 = %d\n", i);
    for (int j = 0; j < dimension2; ++j) {
      for (int k = 0; k < dimension3; ++k) {
        DEBUG("%d ", a[i][j][k]);
      }
      DEBUG("\n");
    }
    DEBUG("\n");
  }
}

class VNEProtectionCPLEXSolver {
 public:
  VNEProtectionCPLEXSolver() { }
  VNEProtectionCPLEXSolver(Graph* physical_topology, Graph* virt_topology,
                           Graph* shadow_virt_topology);

  IloEnv& env()  { return env_; }
  IloModel& model() { return model_; }
  IloCplex& cplex() { return cplex_; }
  IloConstraintArray& constraints() { return constraints_; }
  IloNumArray& preferences() { return preferences_; }
  IloIntVar4dArray& xmn_uv() { return xmn_uv_; }
  void BuildModel();
  void Solve();
 private:

  IloEnv env_;
  IloModel model_;
  IloCplex cplex_;
  IloConstraintArray constraints_;
  IloNumArray preferences_;
  Graph* physical_topology_;
  Graph* virt_topology_;
  Graph* shadow_virt_topology_;
  // Decision variable for edge mapping.
  IloIntVar4dArray xmn_uv_;(env, kTrafficCount);
};
#endif // CPLEX_SOLVER_
