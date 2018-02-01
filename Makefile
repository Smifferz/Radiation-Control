all: main

CXX = g++
CXXFLAGS+=-g -Wall
LDLIBS+=-lstdc++
main: main.o RayBox.o UDPserver.o NavAP.o

	$(CXX) $(CXXFLAGS) -fPIC -o main main.o RayBox.o UDPserver.o NavAP.o
