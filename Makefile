all: main

CXX = g++
CXXFLAGS+=-g -Wall
LDLIBS+=-lstdc++
main: main.o RayBox.o UDPserver.o

	$(CXX) $(CXXFLAGS) -o main main.o RayBox.o UDPserver.o
