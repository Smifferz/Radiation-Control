#include "udpserver.h"
#include "navap.h"
#include "types.h"
#include <iostream>

int main(int argc, char *argv[])
{

  int debug = 0;
  if (argc > 1) {
    for (int i = 1; i < argc; i++) {
      if (strcmp(argv[i], "--debug") == 0) {
        std::cout << "Message: --debug specified, Debug Mode Active." << std::endl;
        debug = 1;
      }
    }
  }

  NavAP *nav = new NavAP(debug);
  while (1) {
    if (nav->check_ping()) {
        nav->NavAPMain();
    }
  }
  delete nav;
  return 0;
}
