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
#include <dirent.h>
#include <stdio.h>

std::list<pid_t> tids;

int main(int argc, char** argv) {
  if(argc != 2) {
    printf("You must supply a PID to get the thread information from.\n  Usage: thread_pids <PID>\n");
    exit(EXIT_FAILURE);
  }

  const std::string filename = "/proc/" + std::string(argv[1]) + "/task";

  DIR* d;
  struct dirent* dir;
  d = opendir(filename.c_str());
  if(d) {
    printf("Getting thread information from PID (%s)\n", argv[1]);
    while((dir = readdir(d)) != NULL) {
      try {
        tids.emplace_back(static_cast<pid_t>(std::stoi(dir->d_name)));
      } catch(const std::exception& e) {
        // Skip the non-number directories
      }
    }
    closedir(d);
  } else {
    printf("Failed to open %s\n", filename.c_str());
  }

  for(auto& tid : tids) {
    printf("%d\n", tid);
  }

  return 0;
}
