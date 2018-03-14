#!/bin/bash

# This wrapper file invokes cmake with the chosen target platform

set -e	# exit if error found


# error handler function
function err {
	echo "Error with: $1"
	echo "Exiting..."
	exit
}
# if there is more than one parameter passed, fail
if [ "$#" -gt "1" ]
then
	err "Num arguments is $#"
fi

# if there are no arguments, default to cross-compilation
if [ "$#" -lt "1" ]
then
	TARGET='arm-linux-gnueabihf-g++'
else
	# set the target compiler
	TARGET=$1
fi

cmake -D CMAKE_CXX_COMPILER=$TARGET -Bbuild -H. 

#
#CROSS-COMPILER=arm-linux-gnueabihf-
#AOCL_COMPILE_CONFIG=$(shell aocl compile-config --arm)
#AOCL_LINK_CONFIG=$(shell aocl link-config --arm)
#
#host_prog : host_prog.o
#	$(CROSS-COMPILER)g++ -o host_prog host_prog.o $(AOCL_LINK_CONFIG)
#
#	host_prog.o : host_prog.cpp
#		$(CROSS-COMPILER)g++ -c host_prog.cpp $(AOCL_COMPILE_CONFIG)
#
