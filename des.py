# Discrete Event Simulator for simulating the arrival and departure of VNs on an
# SN. This simulator was used for the journal version of DRONE. The simulation
# plan, i.e., arrival/departure of VNs with timestamps is provided as a command
# line argument.
import argparse
import heapq
import networkx as nx
import os
import re
import subprocess


class Event:
    def __init__(self, ts, etype, vn_id):
        self.ts = ts
        self.etype = etype
        self.vn_id = vn_id

    def __lt__(self, e):
        return self.ts < e.ts

    def debug_string(self):
        return "ts = " + str(self.ts) + ", etype = " + self.etype + ", vn_id = " + self.vn_id

def execute_one_experiment(executable, pn_topology_file, vn_topology_file, 
                           location_constraint_file):
    process = subprocess.Popen([executable, '--pn_topology_file=' +
                                pn_topology_file, '--vn_topology_file=' +
                                vn_topology_file,
                                '--location_constraint_file=' + location_constraint_file],
                                stdout = subprocess.PIPE, stderr = subprocess.PIPE, shell = False)
    for line in process.stdout:
        print line.strip("\r\n")

def load_csv_graph(topology_file):
    g = nx.Graph()
    with open(topology_file, "r") as f:
        for line in f:
            tokens = line.split(",")
            u, v, bw, cost = int(tokens[1]), int(tokens[2]), int(tokens[5]), int(tokens[4])
            g.add_edge(u, v, {'bw':int(bw), 'cost':int(cost)})
    return g

def write_csv_graph(g, topology_file):
    with open(topology_file, "w") as f:
        edges = g.edges()
        for edge in edges:
            cost = g.get_edge_data(edge[0], edge[1])['cost']
            bw = g.get_edge_data(edge[0], edge[1])['bw']
            f.write(",".join(["0",str(edge[0]), str(edge[1]), "0", str(cost), str(bw), "1"]) + "\n")

def write_util_matrix(sn, util_matrix, out_file):
    with open(out_file, "w") as f:
        for (key, value) in util_matrix.iteritems():
            util = float(value / (value + float(sn.get_edge_data(key[0], key[1])['bw'])))
            if util > 0:
                f.write(",".join([str(key[0]), str(key[1]), str(util)]) + "\n")

def update_graph_capacity(sn, vn, util_matrix, emap_file, semap_file, increase = True):
    sign = 1
    if not increase:
        sign = -1
    with open(emap_file, "r") as f:
        for line in f:
            matches = re.match(
                    "Virtual edge \((\d+), (\d+)\) --> physical edge \((\d+), (\d+)\)", line.strip("\n\r"))
            m, n, u, v = int(matches.group(1)), int(matches.group(2)), int(matches.group(3)), int(matches.group(4))
            if m > n:
                m, n = n, m
            if u > v:
                u, v = v, u
            b_mn = int(vn.get_edge_data(m, n)['bw'])
            sn.get_edge_data(u, v)['bw'] += (sign * b_mn)
            util_matrix[(u, v)] += (-sign * b_mn)

    with open(semap_file, "r") as f:
        for line in f:
            matches = re.match(
                        "Shadow virtual edge of \((\d+), (\d+)\) --> physical edge \((\d+), (\d+)\)", line.strip("\n\r"))
            m, n, u, v = int(matches.group(1)), int(matches.group(2)), int(matches.group(3)), int(matches.group(4))
            if m > n:
                m, n = n, m
            if u > v:
                u, v = v, u
            b_mn = int(vn.get_edge_data(m, n)['bw'])
            sn.get_edge_data(u, v)['bw'] += (sign * b_mn)
            util_matrix[(u, v)] += (-sign * b_mn)

    return sn

def get_embedding_status(status_file):
    ret = ''
    try:
        with open(status_file) as f:
            ret = f.readline().strip("\n\r")
    except IOError:
        pass
    return ret

def get_full_path(cur_directory, file_name):
    if not file_name.startswith("/"):
        return os.path.join(cur_directory, file_name)
    return file_name

def main():
    parser = argparse.ArgumentParser(
                formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--executable", type = str, required = True)
    parser.add_argument("--phys_topology", type = str, default = "sn.txt")
    parser.add_argument("--vnr_directory", type = str, default = "vnr")
    parser.add_argument("--simulation_plan", type = str, 
                        default = "vnr-simulation")
    parser.add_argument("--max_simulation_time", type = int, default = 1000)
    args = parser.parse_args()
    
    current_time = 0
    event_queue = []
    current_directory = os.getcwd()

    with open(args.simulation_plan, "r") as f:
        for line in f:
            tokens = line.split(",")
            ts = int(tokens[0])
            end_time = int(tokens[1])
            vn_id = tokens[2].rstrip("\n")
            e = Event(ts, "arrival", vn_id)
            heapq.heappush(event_queue, e)
            if end_time <= args.max_simulation_time:
                e = Event(end_time, "departure", vn_id)
                heapq.heappush(event_queue, e)
    
    sn = load_csv_graph(args.phys_topology)
    util_matrix = {}
    for u in sn.nodes():
        for v in sn.neighbors(u):
            if u < v:
                util_matrix[(u, v)] = 0.0

    total_vns = 0
    accepted_vns = 0
    rejected_vns = 0
    while not len(event_queue) <= 0:
        e = heapq.heappop(event_queue)
        print e.debug_string()
        vn = load_csv_graph(args.vnr_directory + "/" + e.vn_id)
        if e.etype == "departure":
            # if the embedding of vn_id was not successful at the first place do
            # nothing. This can be checked by reading from $(vn_id).status file.
            # If there was a successful embedding increase graph's capacity.
            status = get_embedding_status(args.vnr_directory + "/" + e.vn_id + ".status")
            print status
            if status == "Optimal" or status == "Successful":
                sn = update_graph_capacity(sn, vn, util_matrix,
                            args.vnr_directory + "/" + e.vn_id + ".emap", 
                            args.vnr_directory + "/" + e.vn_id + ".semap", increase = True)
                write_csv_graph(sn, args.phys_topology)
                write_util_matrix(sn, util_matrix, "sim-data/util-data/util." + str(e.ts))
        elif e.etype == "arrival":
            # run embedding first. if embedding is successful decrease the
            # capacity of SN. Otherwise do nothing.
            total_vns += 1
            pn_topology_file = get_full_path(current_directory, args.phys_topology)
            vn_topology_file = get_full_path(current_directory, 
                                             args.vnr_directory + "/" + e.vn_id)
            location_constraint_file = get_full_path(current_directory,
                                                     args.vnr_directory + "/" +
                                                     e.vn_id + "loc")
            execute_one_experiment(args.executable, pn_topology_file,
                                   vn_topology_file, location_constraint_file)
            status = get_embedding_status(args.vnr_directory  + "/" + e.vn_id + ".status")
            if status == "Optimal" or status == "Successful":
                sn = update_graph_capacity(sn, vn, util_matrix,
                           args.vnr_directory + "/" + e.vn_id + ".emap", 
                           args.vnr_directory + "/" + e.vn_id + ".semap", increase = False)
                write_csv_graph(sn, args.phys_topology)
                write_util_matrix(sn, util_matrix, "sim-data/util-data/util." + str(e.ts))
                accepted_vns += 1
            with open("sim-data/sim-results", "a") as f:
                f.write(",".join([str(e.ts),str(total_vns), str(accepted_vns)]) + "\n")
    print "total = " + str(total_vns) + ", accepted = " + str(accepted_vns)
if __name__ == "__main__":
    main()
    

