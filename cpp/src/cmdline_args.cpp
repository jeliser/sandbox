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

  if(argc == 1) {
    show_usage(argv[0]);
    return(-1);
  }


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
/*
           struct option {
               const char *name;
               int         has_arg;
               int        *flag;
               int         val;
           };
*/
  /* getopt_long stores the option index here. */
  int option_list_size = sizeof(long_options) / sizeof(struct option);
  cout << option_list_size << endl;
  int option_index = 0;

  int c = 0;
  char arg_list[] = "ht:n:P:N:";
  while((c = getopt_long (argc, argv, arg_list, long_options, &option_index)) != -1) {
    switch (c) {
      case 'h': {
        show_usage(argv[0]);
      };  break;
      case 'N': {
        pName = optarg;
      }; break;
      case 'P': {
        pid = atoi(optarg);
      }; break;
      case 'n': {
        tName = optarg;
      }; break;
      case 't': {
        tid = atoi(optarg);
      }; break;
      case '?': {
        bool req_arg = false;
        for(int j = 1; j < sizeof(arg_list); j++) {
          if((arg_list[j] == ':') && (arg_list[j-1] == optopt)) {
            req_arg = true;
          }
        }
        if(req_arg) {
          fprintf(stderr, "Option -%c requires an argument.\n", optopt);
        }
        else if(isprint (optopt)) {
          fprintf(stderr, "Unknown option '-%c`.\n", optopt);
        }
        else {
          fprintf(stderr, "Unknown option character '\\x%x'.\n", optopt);
        }
        return(-1);
      }; break;
      default: {
        printf("ERR:  Unhandled option\n");
      }; break;
    }
  }

  cout << pName << endl;
  cout << pid << endl;
  cout << tName << endl;
  cout << tid << endl;

  return(0);
}
