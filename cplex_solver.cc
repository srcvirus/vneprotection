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
    Graph *physical_topology, Graph *virt_topology, Graph *shadow_virt_topology,
    std::vector<std::vector<int>> *location_constraint) {
  model_ = IloModel(env_);
  cplex_ = IloCplex(model_);
  constraints_ = IloConstraintArray(env_);
  preferences_ = IloNumArray(env_);
  objective_ = IloExpr(env_);
  max_channels_ = 0;
  for (int i = 0; i < physical_topology->node_count(); ++i) {
    auto &neighbors = physical_topology->adj_list()->at(i);
    for (auto &node : neighbors) {
      if (max_channels_ < node.channels) max_channels_ = node.channels;
    }
  }
  physical_topology_ = physical_topology;
  virt_topology_ = virt_topology;
  shadow_virt_topology_ = shadow_virt_topology;
  location_constraint_ = location_constraint;
  x_mn_uv_ = IloIntVar4dArray(env_, virt_topology_->node_count() * 2);
  l_mn_w_ = IloIntVar3dArray(env_, virt_topology_->node_count() * 2);
  l_mn_uv_w_ = IloIntVar5dArray(env_, virt_topology_->node_count() * 2);
  y_m_u_ = IloIntVar2dArray(env_, virt_topology_->node_count() * 2);
  // eta_m_u_ = IloIntVar2dArray(env_, virt_topology_->node_count() * 2);
  l_m_u_ = IloInt2dArray(env_, virt_topology_->node_count() * 2);
  av_uv_w_ = IloInt3dArray(env_, physical_topology_->node_count());

  // Decision variable initialization for virtual network and shadow virtual
  // network.
  for (int m = 0; m < virt_topology_->node_count() * 2; ++m) {
    // Multiply by two for the double number of nodes when considering the
    // shadow network.
    x_mn_uv_[m] = IloIntVar3dArray(env_, virt_topology_->node_count() * 2);
    l_mn_w_[m] = IloIntVar2dArray(env_, virt_topology_->node_count() * 2);
    l_mn_uv_w_[m] = IloIntVar4dArray(env_, virt_topology->node_count() * 2);
    for (int n = 0; n < virt_topology_->node_count() * 2; ++n) {
      x_mn_uv_[m][n] = IloIntVar2dArray(env_, physical_topology_->node_count());
      l_mn_w_[m][n] = IloIntVarArray(env_, max_channels_, 0, 1);
      l_mn_uv_w_[m][n] =
          IloIntVar3dArray(env_, physical_topology_->node_count());
      for (int u = 0; u < physical_topology_->node_count(); ++u) {
        x_mn_uv_[m][n][u] =
            IloIntVarArray(env_, physical_topology_->node_count(), 0, 1);
        l_mn_uv_w_[m][n][u] =
            IloIntVar2dArray(env_, physical_topology_->node_count());
        for (int v = 0; v < physical_topology_->node_count(); ++v) {
          l_mn_uv_w_[m][n][u][v] = IloIntVarArray(env_, max_channels_, 0, 1);
        }
      }
    }
  }

  for (int m = 0; m < virt_topology_->node_count() * 2; ++m) {
    y_m_u_[m] = IloIntVarArray(env_, physical_topology_->node_count(), 0, 1);
    l_m_u_[m] = IloIntArray(env_, physical_topology_->node_count(), 0, 1);
  }

  int offset = virt_topology_->node_count();
  for (int m = 0; m < virt_topology_->node_count(); ++m) {
    for (int u = 0; u < physical_topology_->node_count(); ++u) {
      l_m_u_[m][u] = 0;
      l_m_u_[m + offset][u] = 0;
    }
    auto loc_constraints = location_constraint_->at(m);
    for (auto &u : loc_constraints) {
      l_m_u_[m][u] = 1;
      l_m_u_[m + offset][u] = 1;
    }
  }

  // Initialize channel availability.
  for (int u = 0; u < physical_topology_->node_count(); ++u) {
    av_uv_w_[u] = IloInt2dArray(env_, physical_topology_->node_count());
    for (int v = 0; v < physical_topology_->node_count(); ++v) {
      av_uv_w_[u][v] = IloIntArray(env_, max_channels_, 0, 1);
      for (int w = 0; w < max_channels_; ++w) {
        av_uv_w_[u][v][w] = 0;
      }
    }
    auto &u_neighbors = physical_topology_->adj_list()->at(u);
    for (auto &node : u_neighbors) {
      int v = node.node_id;
      for (int w = 0; w < node.is_channel_available.size(); ++w) {
        if (node.is_channel_available[w]) {
          av_uv_w_[u][v][w] = 1;
        }
      }
    }
  }
}

void VNEProtectionCPLEXSolver::BuildModel() {
  // node_id offset for the shadow virtual topology.
  int offset = virt_topology_->node_count();

  // Constraint: Location constraint of virtual nodes.
  for (int m = 0; m < virt_topology_->node_count(); ++m) {
    for (int u = 0; u < physical_topology_->node_count(); ++u) {
      constraints_.add(y_m_u_[m][u] <= l_m_u_[m][u]);
      constraints_.add(y_m_u_[m + offset][u] <= l_m_u_[m + offset][u]);
    }
  }

  // Constraint: Capacity constraint of physical links.
  for (int u = 0; u < physical_topology_->node_count(); ++u) {
    auto &u_neighbors = physical_topology_->adj_list()->at(u);
    for (auto &end_point : u_neighbors) {
      int v = end_point.node_id;
      int w_uv = end_point.residual_channels;
      IloIntExpr sum(env_);
      IloIntExpr sum_shadow(env_);
      for (int m = 0; m < virt_topology_->node_count(); ++m) {
        auto &m_neighbors = virt_topology_->adj_list()->at(m);
        for (auto &vend_point : m_neighbors) {
          int n = vend_point.node_id;
          int w_mn = vend_point.channels;
          DEBUG("u = %d, v = %d, m = %d, n = %d\n", u, v, m, n);
          DEBUG("u = %d, v = %d, m + offset = %d, n + offset = %d\n", u, v,
                m + offset, n + offset);
          sum += x_mn_uv_[m][n][u][v] * w_mn;
          sum_shadow = x_mn_uv_[m + offset][n + offset][u][v] * w_mn;
        }
      }
      constraints_.add(sum <= w_uv);
      constraints_.add(sum_shadow <= w_uv);
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
          sum += x_mn_uv_[m][n][u][v];
          sum_shadow += x_mn_uv_[m + offset][n + offset][u][v];
        }
      }
      constraints_.add(sum >= 1);
      constraints_.add(sum_shadow >= 1);
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
    constraints_.add(sum == 1);
    constraints_.add(sum_shadow == 1);
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
    constraints_.add(sum <= 1);
  }

  // Constraint: Flow constraint to ensure path connectivity.
  for (int m = 0; m < virt_topology_->node_count(); ++m) {
    auto &m_neighbors = virt_topology_->adj_list()->at(m);
    for (auto &vend_point : m_neighbors) {
      int n = vend_point.node_id;
      // if (m < n) continue;
      for (int u = 0; u < physical_topology_->node_count(); ++u) {
        IloIntExpr sum(env_);
        IloIntExpr sum_shadow(env_);
        auto &u_neighbors = physical_topology_->adj_list()->at(u);
        for (auto &end_point : u_neighbors) {
          int v = end_point.node_id;
          sum += (x_mn_uv_[m][n][u][v] - x_mn_uv_[m][n][v][u]);
          sum_shadow += (x_mn_uv_[m + offset][n + offset][u][v] -
                         x_mn_uv_[m + offset][n + offset][v][u]);
        }
        constraints_.add(sum == (y_m_u_[m][u] - y_m_u_[n][u]));
        constraints_.add(sum_shadow ==
                         (y_m_u_[m + offset][u] - y_m_u_[n + offset][u]));
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
          // constraints_.add(IloIfThen(env_, x_mn_uv_[m][n][u][v] == 1, sum ==
          // 0));
          // constraints_.add(IloIfThen(env_, sum == 0, x_mn_uv_[m][n][u][v] ==
          // 1));
          constraints_.add(IloIfThen(env_, sum > 0, x_mn_uv_[m][n][u][v] == 0));
          constraints_.add(
              IloIfThen(env_, x_mn_uv_[m][n][u][v] == 1, sum == 0));
        }
      }
    }
  }

  // Constraint: No two physical paths having a common physical node be shared
  // between a virtual link and any link from the shadow network. Shadow node
  // mapping should also exclude the nodes used for mapping links of the working
  // virtual network and vice versa.

  for (int u = 0; u < physical_topology_->node_count(); ++u) {
    auto &u_neighbors = physical_topology_->adj_list()->at(u);
    IloIntExpr sum(env_);
    IloIntExpr shadow_node_map_sum(env_);
    IloIntExpr shadow_sum(env_);
    for (int m = 0; m < virt_topology_->node_count(); ++m) {
      auto &m_neighbors = virt_topology_->adj_list()->at(m);
      shadow_node_map_sum += y_m_u_[m + offset][u];
      for (auto &vend_point : m_neighbors) {
        int n = vend_point.node_id;
        for (auto &end_point : u_neighbors) {
          int v = end_point.node_id;
          sum += x_mn_uv_[m][n][u][v];
          shadow_sum += x_mn_uv_[m + offset][n + offset][u][v];
        }
      }
    }
    constraints_.add(IloIfThen(env_, sum > 0, shadow_sum == 0));
    constraints_.add(IloIfThen(env_, shadow_sum > 0, sum == 0));
    // constraints_.add(IloIfThen(env_, shadow_sum == 0, sum > 0));
    constraints_.add(IloIfThen(env_, sum >= 1, shadow_node_map_sum == 0));
    constraints_.add(IloIfThen(env_, shadow_node_map_sum > 0, sum == 0));
    // constraints_.add(IloIfThen(env_, sum >= 1, shadow_node_map_sum == 0));
    for (int m = 0; m < virt_topology_->node_count(); ++m) {
      // constraints_.add(IloIfThen(env_, shadow_sum == 0, y_m_u_[m][u] == 1));
      constraints_.add(IloIfThen(env_, shadow_sum > 0, y_m_u_[m][u] == 0));
      constraints_.add(IloIfThen(env_, y_m_u_[m][u] == 1, shadow_sum == 0));
    }
  }

  // Constraint: A virtual link cannot be assigned a wavelength that is not
  // available for
  // use in the physical link.
  /*
  for (int m = 0; m < virt_topology_->node_count(); ++m) {
    auto& m_neighbors = virt_topology_->adj_list()->at(m);
    for (auto vend_point : m_neighbors) {
      int n = vend_point.node_id;
      for (int u = 0; u < physical_topology_->node_count(); ++u) {
        auto& u_neighbors = physical_topology_->adj_list()->at(u);
        for (auto end_point : u_neighbors) {
          int v = end_point.node_id;
          for (int w = 0; w < max_channels_; ++w) {
            constraints_.add(l_mn_uv_w_[m][n][u][v][w] <= av_uv_w_[u][v][w]);
            constraints_.add(l_mn_uv_w_[m + offset][n + offset][u][v][w] <=
  av_uv_w_[u][v][w]);
          }
        }
      }
    }
  }
  */
  // Constraint: A virtual link should be assigned exactly one wavelength
  for (int m = 0; m < virt_topology_->node_count(); ++m) {
    auto &m_neighbors = virt_topology_->adj_list()->at(m);
    for (auto &vend_point : m_neighbors) {
      int n = vend_point.node_id;
      IloIntExpr sum(env_);
      IloIntExpr shadow_sum(env_);
      for (int w = 0; w < max_channels_; ++w) {
        sum += l_mn_w_[m][n][w];
        shadow_sum += l_mn_w_[m + offset][n + offset][w];
      }
      constraints_.add(sum == 1);
      constraints_.add(shadow_sum == 1);
    }
  }
  // Constraint: A wavelength can be used at most once on any given link in the
  // physical network.
  for (int u = 0; u < physical_topology_->node_count(); ++u) {
    auto &u_neighbors = physical_topology_->adj_list()->at(u);
    for (auto &end_point : u_neighbors) {
      int v = end_point.node_id;
      for (int w = 0; w < end_point.is_channel_available.size(); ++w) {
        IloIntExpr sum(env_);
        IloIntExpr shadow_sum(env_);
        for (int m = 0; m < virt_topology_->node_count(); ++m) {
          auto &m_neighbors = virt_topology_->adj_list()->at(m);
          for (auto &vend_point : m_neighbors) {
            int n = vend_point.node_id;
            sum += l_mn_uv_w_[m][n][u][v][w];
            shadow_sum += l_mn_uv_w_[m + offset][n + offset][u][v][w];
          }
        }
        constraints_.add(sum <= 1);
        constraints_.add(shadow_sum <= 1);
      }
    }
  }
  // Constraint:
  for (int m = 0; m < virt_topology_->node_count(); ++m) {
    auto &m_neighbors = virt_topology_->adj_list()->at(m);
    for (auto vend_point : m_neighbors) {
      int n = vend_point.node_id;
      for (int u = 0; u < physical_topology_->node_count(); ++u) {
        auto u_neighbors = physical_topology_->adj_list()->at(u);
        for (auto &end_point : u_neighbors) {
          int v = end_point.node_id;
          for (int w = 0; w < max_channels_; ++w) {
            constraints_.add(l_mn_uv_w_[m][n][u][v][w] <= l_mn_w_[m][n][w]);
            constraints_.add(l_mn_uv_w_[m][n][u][v][w] <= l_mn_w_[m][n][w]);
            constraints_.add(l_mn_uv_w_[m][n][u][v][w] >=
                             l_mn_w_[m][n][w] + x_mn_uv_[m][n][u][v] - 1);

            constraints_.add(l_mn_uv_w_[m + offset][n + offset][u][v][w] <=
                             l_mn_w_[m + offset][n + offset][w]);
            constraints_.add(l_mn_uv_w_[m + offset][n + offset][u][v][w] >=
                             l_mn_w_[m + offset][n + offset][w] +
                                 x_mn_uv_[m + offset][n + offset][u][v] - 1);
            constraints_.add(l_mn_uv_w_[m + offset][n + offset][u][v][w] <=
                             x_mn_uv_[m + offset][n + offset][u][v]);
          }
        }
      }
    }
  }
  // Objective function.
  for (int m = 0; m < virt_topology_->node_count(); ++m) {
    auto &m_neighbors = virt_topology_->adj_list()->at(m);
    for (auto &vend_point : m_neighbors) {
      int n = vend_point.node_id;
      if (m < n) continue;
      long w_mn = vend_point.channels;
      for (int u = 0; u < physical_topology_->node_count(); ++u) {
        auto &u_neighbors = physical_topology_->adj_list()->at(u);
        for (auto &end_point : u_neighbors) {
          int v = end_point.node_id;
          int cost_uv = end_point.cost;
          DEBUG("u = %d, v = %d, m = %d, n = %d\n", u, v, m, n);
          DEBUG("u = %d, v = %d, m + offset = %d, n + offset = %d\n", u, v,
                m + offset, n + offset);
          objective_ += (x_mn_uv_[m][n][u][v] * cost_uv * w_mn);
          objective_ +=
              (x_mn_uv_[m + offset][n + offset][u][v] * cost_uv * w_mn);
        }
      }
    }
  }
  constraints_.add(objective_ > 0);
  model_.add(constraints_);
  model_.add(IloMinimize(env_, objective_));
}

bool VNEProtectionCPLEXSolver::Solve() {
  // TODO(shihab): Tune parameters of CPLEX solver.
  cplex_.setParam(IloCplex::Threads, 128);
  cplex_.setParam(IloCplex::PreDual, true);
  // cplex_.exportModel("drone.lp");
  bool is_success = cplex_.solve();
  return is_success;
  if (cplex_.getStatus() == IloAlgorithm::Infeasible) {
    IloConstraintArray infeasible(env_);
    IloNumArray preferences(env_);
    infeasible.add(constraints_);
    for (int i = 0; i < infeasible.getSize(); ++i) preferences.add(1.0);
    if (cplex_.refineConflict(infeasible, preferences)) {
      IloCplex::ConflictStatusArray conflict = cplex_.getConflict(infeasible);
      env_.getImpl()->useDetailedDisplay(IloTrue);
      std::cout << "Conflict : " << std::endl;
      for (IloInt i = 0; i < infeasible.getSize(); i++) {
        std::cout << conflict[i] << std::endl;
        if (conflict[i] == IloCplex::ConflictMember)
          std::cout << "Proved  : " << infeasible[i] << std::endl;
        if (conflict[i] == IloCplex::ConflictPossibleMember)
          std::cout << "Possible: " << infeasible[i] << std::endl;
      }
    }
  }
  return is_success;
}
