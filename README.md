# Radiation-Control

This project contains the server side source code for an interface to the Orbiter space simulator
to allow for obstacle avoidance using AABB methods as well as path following via matrix manipulations.

There is another interface on the client side contained elsewhere that presents data to the server and
will manouever the aircraft in the correct manner corresponding to the output from the server.

The application used Linux specific libraries for server side code as it is to be run on an embedded
device.

# Running the application
Running the application is simple. Just change to the project directory and build the source. i.e.
```bash
cd Radiation-Control
make
./main
```
The program can be run in debug mode by specifying  `--debug` in the command line.
```bash
./main --debug
```


# Fin
