#include <iostream>
#include <memory>
#include <atomic>
#include <regex>

#include <unistd.h>
#include <signal.h>
#include <sys/select.h>

#include <ACS.hpp>

// Run loop boolean
std::atomic<bool> running(true);
// regex for sanitizing the input stream
const std::regex re("^(-?\\d{1,})[ ]{1,}(-?\\d{1,})[ ]{1,}(-?\\d{1,})$");

// System handler for reading the SIGINT signal
void sig_handler(int s){
  running = false;
}

// Main application
int main() {
	// Attach a handler for ctrl+c
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = sig_handler;
  sigaction(SIGINT, &sigIntHandler, NULL);
  struct timeval tv {0, 100000};

  // Create an instance of the ACS model
  auto model = std::make_unique<ACS>();

  // Create a simple event loop to service the std::cin and allow the loop to exit
  while(running) {
    fd_set fds;
    FD_ZERO (&fds);
    FD_SET (STDIN_FILENO, &fds);
  
    // select on the file descriptor in the event loop
    if(select (STDIN_FILENO + 1, &fds, NULL, NULL, &tv) != -1) {
      if (FD_ISSET (0, &fds)) {
        // Service the command line user inputs
        std::string cmd;
        std::smatch sm;
        std::getline(std::cin, cmd);
        if(std::regex_match(cmd, sm, re)) {
          model->step(std::stoi(sm[1]), std::stoi(sm[2]), std::stoi(sm[3]));
          std::cout << "Pointing Towards: " << model->get_planet() << " at " << model->get_coordinates() << std::endl;
        }
      }
    }
  }

  return 0;
}
