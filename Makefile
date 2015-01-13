all:
	g++ -O2 -std=c++0x vne_protection.cc -o vne_protection

dbg:
	g++ -DDBG -g -std=c++0x vne_protection.cc -o vne_protection

debug:
	g++ -g -std=c++0x vne_protection.cc -o vne_protection
