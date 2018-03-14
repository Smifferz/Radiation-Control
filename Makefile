all: main

CXX=arm-linux-gnueabihf-g++
CXXFLAGS=-g -H -Wall -Wextra -std=c++11

main: main.o RayBox.o UDPserver.o NavAP.o

	$(CXX) $(CXXFLAGS) -fPIC -o main main.o RayBox.o UDPserver.o NavAP.o
