#include "udpserver.h"
#include "navap.h"
#include "types.h"
#include <iostream>


static void show_help(std::string name)
{
    std::cerr << "Usage: " << name << " <option(s)>"
        << "Options:\n"
        << "\t-h, --help\t\tShow this help\n"
        << "\t-v, --verbose\tSet to verbose mode"
        << "\t-f, --file FILE_LOCATION\tSpecify the OpenCL file location"
        << std::endl;
}


int main(int argc, char *argv[])
{

  int debug = 0;
  std::string file;
  std::string ip;
  if (argc > 1) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-h") || (arg == "--help")) {
            show_help(argv[0]);
            return 0;
        }
        else if ((arg == "-d") || (arg == "--debug")) {
            std::cout << "Message: --debug specified, Debug Mode Active." << std::endl;
            debug = 1;
        }
        else if ((arg == "-f") || (arg == "--file")) {
            if (i + 1 < argc) {
                file = argv[i + 1];
                std::cout << "OpenCL file located at: " << file << std::endl;
            }
            else {
                std::cerr << "--file option requires one argument." << std::endl;
                return 1;
            }
        }
        else if ((arg == "-i") || (arg == "--ip")) {
            if (i + 1 < argc) {
                ip = argv[i + 1];
                std::cout << "IP address of server: " << ip << std::endl;
            }
            else {
                std::cerr << "--ip option requires one argument." << std::endl;
                return 1;
            }
        }
    }
  }
  if (file == "") {
      std::cout << "WARNING: OpenCL file not supplied" << std::endl;
  }
  if (ip == "") {
      std::cerr << "ERROR: IP address not supplied" << std::endl;
      return 1;
  }

  NavAP *nav = new NavAP(ip, debug, file);
  std::cout << "Awaiting incoming connections..." << std::endl;
  while (1) {
    if (nav->check_ping()) {
        nav->NavAPMain();
    }
  }
  delete nav;
  return 0;
}
