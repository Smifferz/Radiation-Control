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


