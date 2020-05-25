#include <cstdio>
#include <string>
#include <chrono>
#include <memory>
#include <QApplication>
#include "robocup_ssl_client.h"

RoboCupSSLClient * init_connection(int port = 10006, string net_ref_address="224.5.23.2", string net_ref_interface="") {
  RoboCupSSLClient *_client = new RoboCupSSLClient(port, net_ref_address, net_ref_interface);  
  if(_client == NULL) {
    printf("[ERROR] Impossible to allocate a client to grsim\n");
    exit(-1);
  }
  
  if(_client->open(true)) {
    printf("[Package handler] Listening to vision system on port %d\n", port);
  } else {
    printf("[ERROR] Cannot listening to vision system on port %d\n", port);
    free(_client);
    exit(-1);
  }

  return _client;
}

void close_connection(RoboCupSSLClient *client) {
  printf("[Package handler] Closing connection to vision system\n");
  client->close();
  free(client);
}

int main(int argc, char ** argv) {
  (void) argc;
  (void) argv;

  RoboCupSSLClient *vision = init_connection(); 

  close_connection(vision);
  return 0;
}
