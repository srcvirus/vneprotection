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

VNEProtectionCPLEXSolver::VNEProtectionCPLEXSolver(
    Graph *physical_topology, Graph *virt_topology,
    Graph *shadow_virt_topology) {
  model_ = IloModel(env_);
  cplex_ = IloCplex(model_);
  constraints_ = IloConstraintArray(env_);
  preferences_ = IloNumArray(env_);
  objective_ = IloExpr(env_);

  physical_topology_ = physical_topology;
  virt_topology_ = virt_topology;
  shadow_virt_topology_ = shadow_virt_topology;
  x_mn_uv_ = IloIntVar4dArray(env_, virt_topology_->node_count() * 2);
  y_m_u_ = IloIntVar2dArray(env_, virt_topology_->node_count() * 2);
  eta_m_u_ = IloIntVar2dArray(env_, virt_topology_->node_count() * 2);

  // Decision variable initialization for virtual network and shadow virtual
  // network.
  for (int m = 0; m < virt_topology_->node_count() * 2; ++m) {
    // Multiply by two for the double number of nodes when considering the
    // shadow network.
    x_mn_uv_[m] = IloIntVar3dArray(env_, virt_topology_->node_count() * 2);

    for (int n = 0; n < virt_topology_->node_count() * 2; ++n) {
      x_mn_uv_[m][n] = IloIntVar2dArray(env_, physical_topology_->node_count());
      for (int u = 0; u < physical_topology_->node_count(); ++u) {
        x_mn_uv_[m][n][u] = IloIntVarArray(
            env_, physical_topology_->node_count(), 0, 1);
      }
    }
  }
  for (int m = 0; m < virt_topology_->node_count() * 2; ++m) {
    y_m_u_[m] = IloIntVarArray(env_, physical_topology_->node_count(), 0, 1);
    eta_m_u_[m] = IloIntVarArray(env_, physical_topology_->node_count(), 0, 1);
  }
}

void VNEProtectionCPLEXSolver::BuildModel() {
  // node_id offset for the shadow virtual topology.
  int offset = virt_topology_->node_count();

  // Constraint: Define eta using constraint on x_mn_uv_
  for (int m = 0; m < virt_topology_->node_count(); ++m) {
    auto &m_neighbors = virt_topology_->adj_list()->at(m);
    for (int u = 0; u < physical_topology_->node_count(); ++u) {
      auto &u_neighbors = physical_topology_->adj_list()->at(u);
      IloIntExpr sum(env_);
      for (auto &vend_point : m_neighbors) {
        int n = vend_point.node_id;
        for (auto &end_point : u_neighbors) {
          int v = end_point.node_id;
          DEBUG("u = %d, v = %d, m = %d, n = %d\n", u, v, m, n);
          sum += x_mn_uv_[m][n][u][v];
        }
      }
      model_.add(IloIfThen(env_, sum >= 1, eta_m_u_[m][u] == 1));
    }
  }

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
          DEBUG("u = %d, v = %d, m = %d, n = %d\n", u, v, m, n);
          DEBUG("u = %d, v = %d, m + offset = %d, n + offset = %d\n",
                u, v, m + offset, n + offset);
          sum += x_mn_uv_[m][n][u][v] * beta_mn;
          sum_shadow = x_mn_uv_[m + offset][n + offset][u][v] * beta_mn;
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
          DEBUG("u = %d, v = %d, m = %d, n = %d\n", u, v, m, n);
          DEBUG("u = %d, v = %d, m + offset = %d, n + offset = %d\n",
                u, v, m + offset, n + offset);
          sum += x_mn_uv_[m][n][u][v];
          sum_shadow += x_mn_uv_[m + offset][n + offset][u][v];
        }
      }
      model_.add(sum >= 1);
      model_.add(sum_shadow >= 1);
    }
  }

  // Constraint: Every virtual node is mapped to exactly one physical node.
  for (int m = 0; m < virt_topology_->node_count(); ++m) {
    IloIntExpr sum(env_);
    IloIntExpr sum_shadow(env_);
    for (int u = 0; u < physical_topology_->node_count(); ++u) {
      DEBUG("u = %d, m = %d,\n", u, m);
      DEBUG("u = %d, m + offset = %d\n", u, m + offset);
      sum += y_m_u_[m][u];
      sum_shadow += y_m_u_[m + offset][u];
    }
    model_.add(sum == 1);
    model_.add(sum_shadow == 1);
  }

  // Constraint: No two virtual nodes are mapped to the same physical node.
  for (int u = 0; u < physical_topology_->node_count(); ++u) {
    IloIntExpr sum(env_);
    for (int m = 0; m < virt_topology_->node_count(); ++m) {
      DEBUG("u = %d, m = %d,\n", u, m);
      DEBUG("u = %d, m + offset = %d\n", u, m + offset);
      sum += y_m_u_[m][u];
      sum += y_m_u_[m + offset][u];
    }
    model_.add(sum <= 1);
  }

  // TODO(shihab): Add location constraint here.

  // Constraint: Flow constraint to ensure path connectivity.
  for (int m = 0; m < virt_topology_->node_count(); ++m) {
    auto &m_neighbors = virt_topology_->adj_list()->at(m);
    for (auto &vend_point : m_neighbors) {
      int n = vend_point.node_id;
      if (m < n)
        continue;
      for (int u = 0; u < physical_topology_->node_count(); ++u) {
        IloIntExpr sum(env_);
        IloIntExpr sum_shadow(env_);
        auto &u_neighbors = physical_topology_->adj_list()->at(u);
        for (auto &end_point : u_neighbors) {
          int v = end_point.node_id;
          sum += (x_mn_uv_[m][n][u][v] - x_mn_uv_[m][n][v][u]);
          sum_shadow += (x_mn_uv_[m + offset][n + offset][u][v] - x_mn_uv_[m + offset][n + offset][v][u]);
        }
        model_.add(sum == (y_m_u_[m][u] - y_m_u_[n][u]));
        model_.add(sum_shadow == (y_m_u_[m + offset][u] - y_m_u_[n + offset][u]));
      }
    }
  }

  // Mutual exclusion constraints.
  // Constraint: A virtual link and its shadow virtual link cannot share the
  // same physical link.
  for (int u = 0; u < physical_topology_->node_count(); ++u) {
    auto &u_neighbors = physical_topology_->adj_list()->at(u);
    for (auto &end_point : u_neighbors) {
      int v = end_point.node_id;
      IloIntExpr sum(env_);
      for (int m = 0; m < virt_topology_->node_count(); ++m) {
        auto &m_neighbors = virt_topology_->adj_list()->at(m);
        for (auto &vend_point : m_neighbors) {
          int n = vend_point.node_id;
          sum += x_mn_uv_[m + offset][n + offset][u][v];
        }
      }
      for (int m = 0; m < virt_topology_->node_count(); ++m) {
        auto &m_neighbors = virt_topology_->adj_list()->at(m);
        for (auto &vend_point : m_neighbors) {
          int n = vend_point.node_id;
          model_.add(IloIfThen(env_, x_mn_uv_[m][n][u][v] == 1, sum == 0));
        }
      }
    }
  }
  // Constraint: No two physical paths having a common physical node be shared
  // between a virtual link and any link from the shadow network.
  for (int u = 0; u < physical_topology_->node_count(); ++u) {
    IloIntExpr sum(env_);
    for (int m_shadow = offset;
         m_shadow < offset + virt_topology_->node_count(); ++m_shadow) {
      sum += eta_m_u_[m_shadow][u];
    }
    for (int m = 0; m < virt_topology_->node_count(); ++m) {
      model_.add(IloIfThen(env_, eta_m_u_[m][u] == 1, sum == 0));
    }
  }
  // Objective function.
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
          DEBUG("u = %d, v = %d, m = %d, n = %d\n", u, v, m, n);
          DEBUG("u = %d, v = %d, m + offset = %d, n + offset = %d\n",
                u, v, m + offset, n + offset);
          objective_ += (x_mn_uv_[m][n][u][v] * cost_uv * beta_mn);
          objective_ +=
              (x_mn_uv_[m + offset][n + offset][u][v] * cost_uv * beta_mn);
        }
      }
    }
  }
  model_.add(objective_ >= 0);
  model_.add(IloMinimize(env_, objective_));
}

bool VNEProtectionCPLEXSolver::Solve() {
  // TODO(shihab): Tune parameters of CPLEX solver.
  cplex_.setOut(env_.getNullStream());
  return cplex_.solve();
}
