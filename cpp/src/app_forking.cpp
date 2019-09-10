// system includes
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <errno.h>
#include <sys/wait.h>
#include <sys/prctl.h>
#include <sys/time.h>
#include <sys/resource.h>

// c++ includes
#include <list>
#include <vector>
#include <thread>
#include <iostream>

std::list<pid_t> pids;

int main() {
  std::list<std::vector<char*>> commands = {
      {(char*)"sleep", (char*)"3", (char*)0},
      {(char*)"sleep", (char*)"5", (char*)0},
      {(char*)"sleep", (char*)"7", (char*)0},
      //{ (char*)"seq", (char*)"3", (char*)"|", (char*)"xargs", (char*)"-I", (char*)"%", (char*)"sh", (char*)"-c",
      //(char*)"'", (char*)"ls", (char*)"-al", (char*)";", (char*)"sleep", (char*)"1", (char*)"'", (char*)0 }, {
      //(char*)"seq", (char*)"3", (char*)0 },
  };

  // Spawn all the commands
  for(auto& command : commands) {
    auto pid = fork();
    if(pid == -1) {
      printf("Failed to fork() process\n");
      exit(-1);
    } else if(pid == 0) {
      // Print out the command
      std::string full_command;
      for(auto& c : command) {
        if(c != NULL) {
          full_command += (std::string(" ") + std::string(c));
        }
      }
      printf("Forked process:%s\n", full_command.c_str());

      // Run the command
      if(execvp(command[0], command.data()) < 0) {
        printf("%d - %s (%s)\n", errno, strerror(errno), command[0]);
      }
      _Exit(0);
    } else {
      pids.push_back(pid);
    }
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(25));

  // Get all pid status
  while(!pids.empty()) {
    for(auto it = pids.begin(); it != pids.end();) {
      int stat;
      // You MUST read the cPID status in order for the <defunct> to get removed
      auto w = waitpid(*it, &stat, WNOHANG);

      // Check the running status of the cPIDS
      if(0 == kill(*it, 0)) {
        struct rusage stat;
        getrusage(*it, &stat);

        printf("RUNNING  - %d  (%f, %f)\n", *it,
               (double)stat.ru_utime.tv_sec + ((double)stat.ru_utime.tv_usec / 1.0E-9),
               (double)stat.ru_stime.tv_sec + ((double)stat.ru_stime.tv_usec / 1.0E-9));

        // std::string proc = "cat /proc/" + std::to_string(*it) + "/stat";
        // system(proc.c_str());

        ++it;
      } else {
        printf("FINISHED - %d\n", *it);
        pids.erase(it++);
      }
    }
    std::cout << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  std::cout << "Closing parent thread" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  return 0;
}
