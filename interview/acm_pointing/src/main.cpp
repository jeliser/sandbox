// system includes
#include <iostream>
#include <memory>
#include <atomic>
#include <regex>

#include <unistd.h>
#include <signal.h>
#include <sys/select.h>

// user includes
#include <ACM.hpp>

// Run loop boolean
std::atomic<bool> running(true);
// regex for sanitizing the input stream
const std::regex re("^(-?\\d{1,})[ ]{1,}(-?\\d{1,})[ ]{1,}(-?\\d{1,})$");
// The user menu
const std::string MENU = "Input ACM movement (x y z) and press enter (ex: 3 -8 5), ctrl+c to exit:";

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
  // The event loop should not block indefinitely, so give it timeout
  struct timeval tv {0, 100000};

  // Create an instance of the ACM model
  auto model = std::make_unique<ACM>();
  // Print the user menu
  std::cout << MENU << std::endl;

  // Create a simple event loop to service the std::cin and allow the loop to exit
  while(running) {
    fd_set fds;
    FD_ZERO (&fds);
    FD_SET (STDIN_FILENO, &fds);
  
    // select on the file descriptor in the event loop
    if(select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) != -1) {
      // Check if the stdin file descriptor was set
      if(FD_ISSET(0, &fds)) {
        // Service the command line user inputs
        std::string cmd;
        std::smatch sm;
        std::getline(std::cin, cmd);
        if(std::regex_match(cmd, sm, re)) {
          // We have a validated input, so step the model
          model->step(std::stoi(sm[1]), std::stoi(sm[2]), std::stoi(sm[3]));
          std::cout << "Pointing Towards: " << model->get_planet() << " at " << model->get_coordinates() << std::endl;
        } else {
          // We have an invalid input, so reprompt the user
          std::cout << "Invalid input: " << cmd << std::endl;
          std::cout << MENU << std::endl;
        }
      }
    }
  }

  return 0;
}
