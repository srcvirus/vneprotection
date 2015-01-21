#include "vne_solution_builder.h"
#include "util.h"

#include <math.h>

void VNESolutionBuilder::PrintWorkingEdgeMapping() {
  auto &cplex = vne_solver_ptr_->cplex();
  auto &x_mn_uv = vne_solver_ptr_->x_mn_uv();
  for (int m = 0; m < virt_topology_->node_count(); ++m) {
    auto &m_neighbors = virt_topology_->adj_list()->at(m);
    for (auto &vend_point : m_neighbors) {
      int n = vend_point.node_id;
      if (m < n)
        continue;
      for (int u = 0; u < physical_topology_->node_count(); ++u) {
        auto &u_neighbors = physical_topology_->adj_list()->at(u);
        for (auto &end_point : u_neighbors) {
          int v = end_point.node_id;
          if (fabs(cplex.getValue(x_mn_uv[m][n][u][v]) - 1) < EPS) {
            printf("Virtual edge (%d, %d) --> physical edge (%d, %d)\n", m, n,
                   u, v);
          }
        }
      }
    }
  }
}

void VNESolutionBuilder::PrintShadowEdgeMapping() {
  auto &cplex = vne_solver_ptr_->cplex();
  auto &x_mn_uv = vne_solver_ptr_->x_mn_uv();
  int offset = virt_topology_->node_count();
  for (int m = 0; m < virt_topology_->node_count(); ++m) {
    auto &m_neighbors = virt_topology_->adj_list()->at(m);
    for (auto &vend_point : m_neighbors) {
      int n = vend_point.node_id;
      if (m < n) 
        continue;
      for (int u = 0; u < physical_topology_->node_count(); ++u) {
        auto &u_neighbors = physical_topology_->adj_list()->at(u);
        for (auto &end_point : u_neighbors) {
          int v = end_point.node_id;
          if (fabs(cplex.getValue(x_mn_uv[m + offset][n + offset][u][v]) - 1) <
              EPS) {
            printf("Shadow virtual edge (%d, %d) --> physical edge (%d, %d)\n",
                   m + offset, n + offset, u, v);
          }
        }
      }
    }
  }
}

void VNESolutionBuilder::PrintWorkingNodeMapping() {
  auto &cplex = vne_solver_ptr_->cplex();
  auto &y_m_u = vne_solver_ptr_->y_m_u();
  for (int m = 0; m < virt_topology_->node_count(); ++m) {
    for (int u = 0; u < physical_topology_->node_count(); ++u) {
      if (fabs(cplex.getValue(y_m_u[m][u]) - 1) < EPS) {
        printf("Virtual node %d --> physical node %d\n", m, u);
      }
    }
  }
}

void VNESolutionBuilder::PrintShadowNodeMapping() {
  auto &cplex = vne_solver_ptr_->cplex();
  auto &y_m_u = vne_solver_ptr_->y_m_u();
  int offset = virt_topology_->node_count();
  for (int m = 0; m < virt_topology_->node_count(); ++m) {
    for (int u = 0; u < physical_topology_->node_count(); ++u) {
      if (fabs(cplex.getValue(y_m_u[m + offset][u]) - 1) < EPS) {
        printf("Shadow virtual node %d --> physical node %d\n", m, u);
      }
    }
  }
}

void VNESolutionBuilder::PrintCost() {
  auto &cplex = vne_solver_ptr_->cplex();
  printf("Cost = %lf\n", cplex.getObjValue());
}
