all: main

CXX=g++
CXXFLAGS=-g -Wall -Wextra -std=c++11

main: main.o RayBox.o UDPserver.o NavAP.o

	$(CXX) $(CXXFLAGS) -fPIC -o main main.o RayBox.o UDPserver.o NavAP.o
