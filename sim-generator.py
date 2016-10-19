# Simulation plan generator for the experiments in journal version of DRONE. VN
# arrival is a poission process, lifetime follows an exponential distribution.
import argparse
import fnss
import networkx as nx
import os
import random
import subprocess
import Queue
from itertools import repeat

def get_physical_topology(gml_file):
    return nx.read_gml(gml_file)

def get_rocketfuel_topology(rocketfuel_file):
    return fnss.parse_rocketfuel_isp_latency(rocketfuel_file)

def write_graph_as_csv(graph, min_bw, max_bw, file_name):
    with open(file_name, 'w') as f:
        for u in range(0, graph.number_of_nodes()):
            neighbors = list(set(graph.neighbors(u)))
            [f.write(",".join(["0",
                               str(u),
                               str(v),
                               "0",
                               "1",
                               str(random.randint(min_bw, max_bw)),
                               "1"]) + "\n") for v in neighbors if u < v]

def get_n_hop_neighbors(graph, s, n):
    q = Queue.Queue()
    q.put(s)
    visited_list = []
    visited = list(repeat(False, graph.number_of_nodes()))
    visited[s] = True
    level = list(repeat(0, graph.number_of_nodes()))
    cur_level = par_level = 0
    while not q.empty():
        if n <= 0:
            break
        u = q.get()
        visited[u] = True
        if cur_level <> level[u]:
            cur_level = level[u]
            n -= 1
        for v in graph.neighbors(u):
            if not visited[v]:
                q.put(v)
                level[v] = level[u] + 1
                visited_list.append(v)
    ret_list = list(set(visited_list))
    if len(ret_list) > 45:
        ret_list = ret_list[:44]
    return ret_list

def generate_location_constraint(vn, phys_topology, loc_file):
    with open(loc_file, 'w') as f:
        for i in range(0, vn.number_of_nodes()):
            rand_pnode = random.randint(0, phys_topology.number_of_nodes() - 1)
            locations = map(lambda x: str(x), get_n_hop_neighbors(phys_topology, rand_pnode, 4))
            f.write(",".join([str(i)] + locations) + "\n")
def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--vn_arrival_rate', type=float, default=0.05)
    parser.add_argument('--max_simulation_time', type=int, default=1000)
    parser.add_argument('--vn_lifetime', type=int, default=100)
    parser.add_argument('--seed', type=int, default=0x5414ab)
    parser.add_argument('--physical_topology_file', type=str,
                        default='6461.latencies.intra')
    args = parser.parse_args()

    random.seed(args.seed)

    phys_topology = get_rocketfuel_topology(args.physical_topology_file)
    write_graph_as_csv(phys_topology, 35000, 40000, "sn.txt")
    times = []
    current_time = 0
    while current_time < args.max_simulation_time:
        wait_time = int(random.expovariate(args.vn_arrival_rate)) + 1
        duration = int(random.expovariate(1.0 / args.vn_lifetime)) + 1
        times.append((current_time, current_time + duration))
        current_time += wait_time
    
    vn_node_counts = [random.randint(4, 8) for i in range(0, len(times))]
    vns = []
    for vn_node_count in vn_node_counts:
        vns.append(nx.gnp_random_graph(vn_node_count, 0.5, seed = args.seed))

    with open('vnr-simulation', 'w') as f:
        vn_id = 0
        for time_point in times:
            current_time, ends_at = time_point
            vn_file = "vn" + str(vn_id)
            vn = vns[vn_id]
            f.write(
                ",".join([str(current_time), str(ends_at), vn_file]) + "\n")
            write_graph_as_csv(vn, 8000, 10000, "vnr/" + vn_file)
            loc_file = vn_file + "loc"
            generate_location_constraint(vn, phys_topology, "vnr/" + loc_file)
            vn_id += 1

if __name__ == '__main__':
    main()
