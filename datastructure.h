#ifndef DATASTRUCTURE_H_
#define DATASTRUCTURE_H_

#include <list>
#include <string>
#include <sstream>
#include <vector>
#include <math.h>
#include <map>
#include <memory>
#include <stdlib.h>

#define INF 99999999
#define MAXN 1000
#define NIL -1
#define EPS 1e-9

struct edge_endpoint {
  int node_id;
  long bandwidth;
  long residual_bandwidth;
  int delay;
  int cost;
  edge_endpoint(int node_id, long bw, int delay, int cost)
      : node_id(node_id), bandwidth(bw), delay(delay), residual_bandwidth(bw),
        cost(cost) {}
  std::string GetDebugString() {
    return "ndoe_id = " + std::to_string(node_id) + ", bandwidth = " +
           std::to_string(bandwidth) + ", delay = " + std::to_string(delay) +
           ", cost = " + std::to_string(cost);
  }
};

class Graph {
public:
  Graph() {
    adj_list_ = std::unique_ptr<std::vector<std::vector<edge_endpoint> > >(
        new std::vector<std::vector<edge_endpoint> >);
    node_count_ = edge_count_ = 0;
  }

  // Accessor methods.
  int node_count() const { return node_count_; }
  int edge_count() const { return edge_count_; }
  const std::vector<std::vector<edge_endpoint> > *adj_list() const {
    return static_cast<const std::vector<std::vector<edge_endpoint> > *>(
        adj_list_.get());
  }

  // u and v are 0-based identifiers of an edge endpoint. An edge is
  // bi-directional, i.e., calling Graph::add_edge with u = 1, v = 3 will add
  // both (1, 3) and (3, 1) in the graph.
  int add_edge(int u, int v, long bw, int delay, int cost) {
    if (adj_list_->size() < u + 1)
      adj_list_->resize(u + 1);
    if (adj_list_->size() < v + 1)
      adj_list_->resize(v + 1);
    adj_list_->at(u).push_back(edge_endpoint(v, bw, delay, cost));
    adj_list_->at(v).push_back(edge_endpoint(u, bw, delay, cost));
    ++edge_count_;
    node_count_ = adj_list_->size();
  }

  std::string GetDebugString() {
    std::string ret_string = "node_count = " + std::to_string(node_count_);
    ret_string += ", edge_count = " + std::to_string(edge_count_) + "\n";
    for (int i = 0; i < node_count_; ++i) {
      auto &neighbors = adj_list_->at(i);
      ret_string += std::to_string(i) + " --> ";
      for (auto &neighbor : neighbors) {
        ret_string += " (" + neighbor.GetDebugString() + ")";
      }
      ret_string += "\n";
    }
    return ret_string;
  }

private:
  std::unique_ptr<std::vector<std::vector<edge_endpoint> > > adj_list_;
  int node_count_, edge_count_;
};
#endif // MIDDLEBOX_PLACEMENT_SRC_DATASTRUCTURE_H_
