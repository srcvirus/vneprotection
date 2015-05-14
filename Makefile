#LIB_PATHS = -L/home/sr2chowd/cplex/cplex/lib/x86-64_sles10_4.1/static_pic -L/home/sr2chowd/cplex/concert/lib/x86-64_sles10_4.1/static_pic

#INCLUDE_PATHS = -I/home/sr2chowd/cplex/cplex/include -I/home/sr2chowd/cplex/concert/include
LIB_PATHS=
INCLUDE_PATHS=
LIBS = -lilocplex -lconcert -lcplex -lm -lpthread  -DIL_STD

FILES = vne_protection.cc cplex_solver.cc util.cc vne_solution_builder.cc

all:
	g++ -O3 -std=c++0x $(LIB_PATHS) $(INCLUDE_PATHS) $(FILES) $(LIBS) -o vne_protection

dbg:
	g++ -DDBG -g -std=c++0x $(LIB_PATHS) $(INCLUDE_PATHS) $(FILES) $(LIBS) -o vne_protection

debug:
	g++ -g -std=c++0x $(LIB_PATHS) $(INCLUDE_PATHS) $(FILES) $(LIBS) -o vne_protection
