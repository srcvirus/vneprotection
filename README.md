# CPLEX Implementation for Virtual Network Embedding with 1 + 1 Network Protection

## Dependencies

The implementation uses IBM ILOG CPLEX C++ API (version 12.5 of CPLEX Studio).
We assume CPLEX is installed in /opt/ibm/ILOG/CPLEX_Studio125. In case CPLEX is
installed in a different directory replace /opt/ibm/ILOG/CPLEX_Studio125 in the
Makefile with the CPLEX installation directory.

## How to run
```
$ make
$ ./vne_protection --pn_topology_file=<physical_network_topology>\ 
                   --vn_topology_file=<virtual_network_topology>\
                   --location_constraint_file=<location_constraint_file>
```

Two example physical (test_pn.topo) and virtual (test_vn.topo) network topology
files are provided with the distribution. A sample location constraint file is
provided as well (test_location.txt).

## Input file format

A topology file contains the list of edges. Each line contains a description of
an edge in a comma separated value (CSV) format. Format of a line is as follows:
```
<LinkID>,<SourceNodeId>,<DestinationNodeId>,<PeerID>,<Cost>,<Bandwidth>,<Latency>
```
Where,
  * LinkID = Id of a link. Ignored for both physical and virtual topology.
  * SourceNodeId = 0-based node index of the source of the link*
  * DestinationNodeId = 0-based node index of the destination of the link*
  * PeerID = Ignored
  * Cost = Cost of provisioning unit bandwidth on this link. Cost is ignored for
           virtual links.
  * Bandwidth = Available bandwidth of a physical link. In case of virtual link,
                this is the bandwidth requirement
  * Delay = Latency of a physical link. In case of virtual link, this is the
            maximum delay requirement for the virtual link.

A location constraint file contains as many lines as the number of virtual
nodes. Each line is a comma separated list of values. The first value indicates
the id of a virtual node followed by the ids of physical nodes where this
virtual node can be mapped.

*Nodes are numberded from `0 ... (n - 1)` in a network with `n` nodes.

## Output Files

Currently the solver prints output to the standard output and writes them to
different output files as well. Each output file is prefixed with the
physical topology file name and has the following suffixes based on the
contents:

* .cost = solution cost
* .nmap = node mapping
* .emap = edge mapping
* .snmap = shadow node mapping
* .semap = shadwo edge mapping
