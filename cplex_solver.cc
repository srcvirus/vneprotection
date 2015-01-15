#include "cplex_solver.h"

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

void PrintIloInt3dArray(IloInt3dArray &a, int dimension1,
                        int dimension2, int dimension3, string name) {
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
VNEProtectionCPLEXSolver::VNEProtectionCPLEXSolver(
    Graph *physical_topology, Graph *virt_topology,
    Graph *shadow_virt_topology) {
  model_ = IloModel(env_);
  cplex_ = IloCplex(model_);
  constraints_ = IloConstraintArray(env_);
  preferences_ = IloNumArray(env_);

  physical_topology_ = physical_topology;
  virt_topology_ = virt_topology;
  shadow_virt_topology_ = shadow_virt_topology;
  xmn_uv_ = IloIntVar4dArray(
      env_, virt_topology_->node_count() + shadow_virt_topology_->node_count());
  // Decision variable initialization for virtual network.
  for (int m = 0; m < virt_topology_->node_count(); ++m) {
    auto &m_neighbors = virt_topology->adj_list()->at(m);
    xmn_uv_[m] = IloIntVar3dArray(env_, m_neighbors.size());
    for (int n = 0; n < m_neighbors.size(); ++n) {
      xmn_uv_[m][n] = IloIntVar2dArray(env_, physical_topology_->node_count());
      for (int u = 0; u < physical_topology_->node_count(); ++u) {
        xmn_uv_[m][n][u] =
            IloIntVarArray(env_, physical_topology_->adj_list()->at(u).size());
      }
    }
  }

  // Decision variable initialization for shadow virtual network.
  int offset = virt_topology_->node_count();
  for (int m = offset; m < offset + shadow_virt_topology_->node_count(); ++m) {
    auto &m_neighbors = shadow_virt_topology->adj_list()->at(m - offset);
    xmn_uv_[m] = IloIntVar3dArray(env_, m_neighbors.size() + offset);
    for (int n = 0; n < m_neighbors.size(); ++n) {
      xmn_uv_[m][n] = IloIntVar2dArray(env_, physical_topology_->node_count());
      for (int u = 0; u < physical_topology_->node_count(); ++u) {
        xmn_uv_[m][n][u] =
            IloIntVarArray(env_, physical_topology_->adj_list()->at(u).size());
      }
    }
  }
}

void VNEProtectionCPLEXSolver::BuildModel() {
  // node_id offset for the shadow virtual topology.
  int offset = virt_topology_->node_count();

  // Constraint: Capacity constraint of physical links.
  for (int u = 0; u < physical_topology_->node_count(); ++u) {
    auto &u_neighbors = physical_topology_->adj_list()->at(u);
    for (auto &end_point : u_neighbors) {
      int v = end_point.node_id;
      int beta_uv = end_point.bandwidth;
      IloIntExpr sum(env_);
      IloIntExpr sum_shadow(env_);
      for (int m = 0; m < virt_topology_->node_count(); ++m) {
        auto &m_neighbors = virt_topology_->adj_list()->at(m);
        for (auto &vend_point : m_neighbors) {
          int n = vend_point.node_id;
          int beta_mn = vend_point.bandwidth;
          sum += xmn_uv_[m][n][u][v] * beta_mn;
          sum_shadow = xmn_uv_[m + offset][n + offset][u][v] * beta_mn;
        }
      }
      model_.add(sum <= beta_uv);
      model_.add(sum_shadow <= beta_uv);
    }
  }

  // Constraint: Every virtual link is mapped to one or more physical links.
  for (int m = 0; m < virt_topology_->node_count(); ++m) {
    auto &m_neighbors = virt_topology_->adj_list()->at(m);
    for (auto &vend_point : m_neighbors) {
      int n = vend_point.node_id;
      IloIntExpr sum(env_);
      IloIntExpr sum_shadow(env_);
      for (int u = 0; u < physical_topology_->node_count(); ++u) {
        auto &u_neighbors = physical_topology_->adj_list()->at(u);
        for (auto &end_point : u_neighbors) {
          int v = end_point.node_id;
          sum += xmn_uv_[m][n][u][v];
          sum_shadow += xmn_uv_[m + offset][n + offset][u][v];
        }
      }
      model_.add(sum >= 1);
      model_.add(sum_shadow >= 1);
    }
  }

  // Objective function.
  IloExpr objective;
  for (int m = 0; m < virt_topology_->node_count(); ++m) {
    auto &m_neighbors = virt_topology_->adj_list()->at(m);
    for (auto &vend_point : m_neighbors) {
      int n = vend_point.node_id;
      long beta_mn = vend_point.bandwidth;
      for (int u = 0; u < physical_topology_->node_count(); ++u) {
        auto &u_neighbors = physical_topology_->adj_list()->at(u);
        for (auto &end_point : u_neighbors) {
          int v = end_point.node_id;
          int cost_uv = end_point.cost;
          objective += (xmn_uv_[m][n][u][v] * cost_uv * beta_mn);
          objective +=
              (xmn_uv_[m + offset][v + offset][u][v] * cost_uv * beta_mn);
        }
      }
    }
  }
  model_.add(objective >= 0);
  model_.add(IloMinimize(env_, objective));
}

void VNEProtectionCPLEXSolver::Solve() {
  try {
    cplex_.solve();
  }
  catch (IloException & e) {
    printf("Exception caught: %s\n", e.getMessage());
  }
}
