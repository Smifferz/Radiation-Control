#include "UDPserver.h"
#include "NavAP.h"
#include "types.h"

int main(int argc, char *argv[])
{

  int debug = 0;
  if (argc > 1)
    debug = atoi(argv[1]);

  NavAP *nav = new NavAP(debug);
  int data = 0;
  while(1) {
    // Attempt to perform transfer
    //std::string s = std::to_string(data);
    //const char *pchar = s.c_str();
    //VECTOR3 result;
    //server->perform_transfer(data, 0, debug, result);
    //printf("Data is %d\n", data);
    //data++;
    // Attempt to make a connection with the client
    if (nav->check_ping()) {
        while(1) {
        nav->NavAPMain();
        sleep(1);
        }
    }
  }
  delete nav;
  return 0;
}
