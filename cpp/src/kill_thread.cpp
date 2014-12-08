/****
**
** Test app for testing out the termination of a running thread.
**
******/

#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>

#include <iostream>
#include <string>

using namespace std;

static void show_usage(char * prog) {
  cerr << "Usage: " << prog << " <option(s)>\n"
       << "Options:\n"
       << "\t-P,--pid PROCESS_PID\tSpecify the process's PID\n"
       << "\t-N,--pname PROCESS_NAME\tSpecify the process's NAME\n"
       << "\t-t,--tid THREAD_PID\tSpecify the thread's PID\n"
       << "\t-n,--tname THREAD_NAME\tSpecify the thread's NAME\n"
       << "\t-h,--help\t\tShow this help message\n\n";
}


int main(int argc, char * argv[]) {

  string pName = "";
  int pid = -1;
  string tName  = "";
  int tid = -1;

  int verbose = 0;

  static struct option long_options[] =
    {
      /* These options set a flag. */
      {"verbose", no_argument,       &verbose, 1},
      {"brief",   no_argument,       &verbose, 0},
      /* These options don’t set a flag.
         We distinguish them by their indices. */
      {"help",    no_argument,       0, 'h'},
      {"pid",     required_argument, 0, 'P'},
      {"pname",   required_argument, 0, 'N'},
      {"tid",     required_argument, 0, 't'},
      {"tname",   required_argument, 0, 'n'},
    };
  /* getopt_long stores the option index here. */
  int option_index = 0;

  int c = 0;
  while((c = getopt_long (argc, argv, "ht:n:P:N:", long_options, &option_index)) != -1) {
    switch (c) {
      case 'h':
        show_usage(argv[0]);
        break;
      case 'P':
        pName = optarg;
        break;
      case 'N':
        pid = atoi(optarg);
        break;
      case 't':
        tid = atoi(optarg);
        break;
      case 'n':
        tName = optarg;
        break;
      default: break;
    }
  }


  cout << pName << endl;
  cout << pid << endl;
  cout << tName << endl;
  cout << tid << endl;

/*
  if(tgkill(tgid, tid, sig)) {
    printf("Failed to kill: %s (%d)", threadName.c_str(), tid);
  }
*/

  return(0);
}
