/* demo_inotify.c

   Demonstrate the use of the inotify API.

   Usage: demo_inotify pathname...

   The program monitors each of the files specified on the command line for all
   possible file events.

   This program is Linux-specific. The inotify API is available in Linux 2.6.13
   and later.
*/
#include <unistd.h>
#include <signal.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/inotify.h>

volatile sig_atomic_t done = 0;
void signalHandler(int sig) {  // can be called asynchronously
  done = 1;                    // set flag
}

static void /* Display information from inotify_event structure */
displayInotifyEvent(struct inotify_event* i) {
  printf("    wd =%2d; ", i->wd);
  if(i->cookie > 0) printf("cookie =%4d; ", i->cookie);

  if(i->len > 0) printf(" name = %s  ", i->name);

  printf("mask = ");
  if(i->mask & IN_ACCESS) printf("IN_ACCESS ");
  if(i->mask & IN_ATTRIB) printf("IN_ATTRIB ");
  if(i->mask & IN_CLOSE_NOWRITE) printf("IN_CLOSE_NOWRITE ");
  if(i->mask & IN_CLOSE_WRITE) printf("IN_CLOSE_WRITE ");
  if(i->mask & IN_CREATE) printf("IN_CREATE ");
  if(i->mask & IN_DELETE) printf("IN_DELETE ");
  if(i->mask & IN_DELETE_SELF) printf("IN_DELETE_SELF ");
  if(i->mask & IN_IGNORED) printf("IN_IGNORED ");
  if(i->mask & IN_ISDIR) printf("IN_ISDIR ");
  if(i->mask & IN_MODIFY) printf("IN_MODIFY ");
  if(i->mask & IN_MOVE_SELF) printf("IN_MOVE_SELF ");
  if(i->mask & IN_MOVED_FROM) printf("IN_MOVED_FROM ");
  if(i->mask & IN_MOVED_TO) printf("IN_MOVED_TO ");
  if(i->mask & IN_OPEN) printf("IN_OPEN ");
  if(i->mask & IN_Q_OVERFLOW) printf("IN_Q_OVERFLOW ");
  if(i->mask & IN_UNMOUNT) printf("IN_UNMOUNT ");
  printf("\n");
}

#define EVENT_SIZE (sizeof(struct inotify_event))
#define EVENT_BUF_LEN (1024 * (EVENT_SIZE + 16))

int main() {
  int length;
  int fd;
  int wd;
  char buffer[EVENT_BUF_LEN];
  const char* filepath = "/tmp";

  // signal(SIGINT, signalHandler);

  /*creating the INOTIFY instance*/
  fd = inotify_init();

  /*checking for error*/
  if(fd < 0) {
    perror("inotify_init");
  }

  /*adding the “/tmp” directory into watch list. Here, the suggestion is to
   * validate the existence of the directory before adding into monitoring
   * list.*/
  wd = inotify_add_watch(fd, filepath, IN_ALL_EVENTS);
  printf("Monitoring inotify events on %s\n", filepath);

  for(int i = 0; !done; i = 0) {
    /*read to determine the event change happens on “/tmp” directory. Actually
     * this read blocks until the change event occurs*/
    length = read(fd, buffer, EVENT_BUF_LEN);

    /*checking for error*/
    if(length < 0) {
      perror("read");
    }

    /*actually read return the list of change events happens. Here, read the
     * change event one by one and process it accordingly.*/
    while(i < length) {
      struct inotify_event* event = (struct inotify_event*)&buffer[i];
      displayInotifyEvent(event);
      i += EVENT_SIZE + event->len;
    }
  }

  /*removing the “/tmp” directory from the watch list.*/
  inotify_rm_watch(fd, wd);

  /*closing the INOTIFY instance*/
  close(fd);
}
