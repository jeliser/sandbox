// basic file operations
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <signal.h>

volatile sig_atomic_t done = 0;
void signalHandler(int sig){ // can be called asynchronously
  done = 1; // set flag
}


int main () {
  signal(SIGINT, signalHandler);

  std::ofstream myfile;
  myfile.open ("/tmp/example.log");

  for( ; !done; ) {
    std::this_thread::sleep_for (std::chrono::seconds(1));  
    myfile << "Writing this to a file." << std::endl;
    std::cout << myfile.good() << "  " << myfile.fail() << "  " << myfile.bad() << std::endl;
  }

  myfile.close();

  return 0;
}
