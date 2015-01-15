LIB_PATHS = -L/opt/ibm/ILOG/CPLEX_Studio125/cplex/lib/x86-64_sles10_4.1/static_pic -L/opt/ibm/ILOG/CPLEX_Studio125/concert/lib/x86-64_sles10_4.1/static_pic

INCLUDE_PATHS = -I/opt/ibm/ILOG/CPLEX_Studio125/cplex/include -I/opt/ibm/ILOG/CPLEX_Studio125/concert/include

LIBS = -lilocplex -lconcert -lcplex -lm -lpthread  -DIL_STD

FILES = vne_protection.cc cplex_solver.cc util.cc

all:
	g++ -O2 -std=c++0x $(LIB_PATHS) $(INCLUDE_PATHS) $(FILES) $(LIBS) -o vne_protection

dbg:
	g++ -DDBG -g -std=c++0x $(LIB_PATHS) $(INCLUDE_PATHS) $(FILES) $(LIBS) -o vne_protection

debug:
	g++ -g -std=c++0x $(LIB_PATHS) $(INCLUDE_PATHS) $(FILES) $(LIBS) -o vne_protection
