#include "vne_solution_builder.h"
#include "util.h"

#include <math.h>

void VNESolutionBuilder::PrintWorkingEdgeMapping(const char *filename) {
  FILE *outfile = NULL;
  if (filename) outfile = fopen(filename, "w");
  auto &cplex = vne_solver_ptr_->cplex();
  auto &x_mn_uv = vne_solver_ptr_->x_mn_uv();
  auto &l_mn_uv_w = vne_solver_ptr_->l_mn_uv_w();
  int max_channels = vne_solver_ptr_->max_channels();
  for (int m = 0; m < virt_topology_->node_count(); ++m) {
    auto &m_neighbors = virt_topology_->adj_list()->at(m);
    for (auto &vend_point : m_neighbors) {
      int n = vend_point.node_id;
      if (m < n) continue;
      for (int u = 0; u < physical_topology_->node_count(); ++u) {
        auto &u_neighbors = physical_topology_->adj_list()->at(u);
        for (auto &end_point : u_neighbors) {
          int v = end_point.node_id;
          for (int w = 0; w < max_channels; ++w) {
            if (fabs(cplex.getValue(x_mn_uv[m][n][u][v]) - 1) < EPS &&
                fabs(cplex.getValue(l_mn_uv_w[m][n][u][v][w]) - 1) < EPS) {
              printf(
                  "Virtual edge (%d, %d) --> physical edge (%d, %d): Channel "
                  "%d\n",
                  m, n, u, v, w);
              if (outfile) {
                fprintf(outfile,
                        "Virtual edge (%d, %d) --> physical edge (%d, %d): "
                        "Channel %d\n",
                        m, n, u, v, w);
              }
            }
          }
        }
      }
    }
  }
  if (outfile) fclose(outfile);
}

void VNESolutionBuilder::PrintShadowEdgeMapping(const char *filename) {
  FILE *outfile = NULL;
  if (filename) outfile = fopen(filename, "w");
  auto &cplex = vne_solver_ptr_->cplex();
  auto &x_mn_uv = vne_solver_ptr_->x_mn_uv();
  auto &l_mn_uv_w = vne_solver_ptr_->l_mn_uv_w();
  int max_channels = vne_solver_ptr_->max_channels();
  int offset = virt_topology_->node_count();
  for (int m = 0; m < virt_topology_->node_count(); ++m) {
    auto &m_neighbors = virt_topology_->adj_list()->at(m);
    for (auto &vend_point : m_neighbors) {
      int n = vend_point.node_id;
      if (m < n) continue;
      for (int u = 0; u < physical_topology_->node_count(); ++u) {
        auto &u_neighbors = physical_topology_->adj_list()->at(u);
        for (auto &end_point : u_neighbors) {
          int v = end_point.node_id;
          for (int w = 0; w < max_channels; ++w) {
            if ((fabs(cplex.getValue(x_mn_uv[m + offset][n + offset][u][v]) -
                      1) < EPS) &&
                (fabs(cplex.getValue(
                          l_mn_uv_w[m + offset][n + offset][u][v][w]) -
                      1) < EPS)) {
              printf(
                  "Shadow virtual edge of (%d, %d) --> physical edge (%d, %d): "
                  "Channel %d\n",
                  m, n, u, v, w);
              if (outfile) {
                fprintf(outfile,
                        "Shadow virtual edge of (%d, %d) --> physical "
                        "edge (%d, %d): Channel %d\n",
                        m, n, u, v, w);
              }
            }
          }
        }
      }
    }
  }
  if (outfile) fclose(outfile);
}

void VNESolutionBuilder::PrintWorkingNodeMapping(const char *filename) {
  FILE *outfile = NULL;
  if (filename) outfile = fopen(filename, "w");
  auto &cplex = vne_solver_ptr_->cplex();
  auto &y_m_u = vne_solver_ptr_->y_m_u();
  for (int m = 0; m < virt_topology_->node_count(); ++m) {
    for (int u = 0; u < physical_topology_->node_count(); ++u) {
      if (fabs(cplex.getValue(y_m_u[m][u]) - 1) < EPS) {
        printf("Virtual node %d --> physical node %d\n", m, u);
        if (outfile) {
          fprintf(outfile, "Virtual node %d --> physical node %d\n", m, u);
        }
      }
    }
  }
  if (outfile) fclose(outfile);
}

void VNESolutionBuilder::PrintShadowNodeMapping(const char *filename) {
  FILE *outfile = NULL;
  if (filename) outfile = fopen(filename, "w");
  auto &cplex = vne_solver_ptr_->cplex();
  auto &y_m_u = vne_solver_ptr_->y_m_u();
  int offset = virt_topology_->node_count();
  for (int m = 0; m < virt_topology_->node_count(); ++m) {
    for (int u = 0; u < physical_topology_->node_count(); ++u) {
      if (fabs(cplex.getValue(y_m_u[m + offset][u]) - 1) < EPS) {
        printf("Shadow virtual node of %d --> physical node %d\n", m, u);
        if (outfile) {
          fprintf(outfile, "Shadow virtual node of %d --> physical node %d\n",
                  m, u);
        }
      }
    }
  }
  if (outfile) fclose(outfile);
}

void VNESolutionBuilder::PrintWaveLengthMapping(const char *filename) {
  FILE *outfile = NULL;
  if (filename) outfile = fopen(filename, "w");
  auto &cplex = vne_solver_ptr_->cplex();
  auto &l_mn_w = vne_solver_ptr_->l_mn_w();
  int max_channels = vne_solver_ptr_->max_channels();
  int offset = virt_topology_->node_count();
  for (int m = 0; m < virt_topology_->node_count(); ++m) {
    auto &m_neighbors = virt_topology_->adj_list()->at(m);
    for (auto vend_point : m_neighbors) {
      int n = vend_point.node_id;
      if (m < n) continue;
      for (int w = 0; w < max_channels; ++w) {
        if (fabs(cplex.getValue(l_mn_w[m][n][w]) - 1) < EPS) {
          printf("Virtual link (%d,%d) -> wavelength %d\n", m, n, w);
          if (outfile) {
            fprintf(outfile, "Virtual link (%d, %d) --> wavelength %d\n", m, n,
                    w);
          }
        }
        if (fabs(cplex.getValue(l_mn_w[m + offset][n + offset][w]) - 1) < EPS) {
          printf("Shadow virtual link (%d,%d) --> wavelength %d\n", m, n, w);
          if (outfile) {
            fprintf(outfile, "Shadow virtual link (%d,%d) --> wavelength %d\n",
                    m, n, w);
          }
        }
      }
    }
  }
  if (outfile) fclose(outfile);
}

void VNESolutionBuilder::PrintCost(const char *filename) {
  FILE *outfile = NULL;
  if (filename) outfile = fopen(filename, "w");
  auto &cplex = vne_solver_ptr_->cplex();
  printf("Cost = %lf\n", cplex.getObjValue());
  if (outfile) {
    fprintf(outfile, "Cost = %lf\n", cplex.getObjValue());
  }
}
