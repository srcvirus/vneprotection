#include "datastructure.h"
#include "io.h"
#include "util.h"

const std::string kUsage = 
  "./vne_protection "
  "--pn_topology_file=<pn_topology_file>\n\t"
  "--vn_topology_file=<vn_topology_file>";

int main(int argc, char *argv[]) {
  using std::string;
  auto arg_map = ParseArgs(argc, argv);
  string pn_topology_filename = "";
  string vn_topology_filename = "";
  for (auto argument : *arg_map) {
    if (argument.first == "--pn_topology_file") {
      pn_topology_filename = argument.second;
    } else if (argument.first == "--vn_topology_file") {
      vn_topology_filename = argument.second;
    } else {
      printf("Invalid command line option: %s\n", argument.first.c_str());
      return 1;
    }
  }
  auto physical_topology = InitializeTopologyFromFile(pn_topology_filename.c_str());
  auto virt_topology = InitializeTopologyFromFile(vn_topology_filename.c_str());
  auto shadow_virt_topology = InitializeTopologyFromFile(vn_topology_filename, c_str());
  return 0;
}
