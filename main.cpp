#include "UDPserver.h"
#include "RayBox.h"
#include "types.h"

int main(int argc, char *argv[])
{

  std::string address = argv[1];
  int debug = 0;
  if (argc > 2)
    debug = atoi(argv[2]);

  UDPserver *server = new UDPserver(address);
  int data = 0;
  while(1) {
    // Attempt to perform transfer
    std::string s = std::to_string(data);
    const char *pchar = s.c_str();
    VECTOR3 result;
    server->perform_transfer(pchar, debug, result);
    printf("Data is %d\n", data);
    data++;
  }
  return 0;
}
