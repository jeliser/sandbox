#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <signal.h>
#include <unistd.h>
#include <string.h>

volatile std::atomic<bool> running;
volatile std::atomic<bool> consume;

std::mutex mutex;
std::condition_variable cv;

void consumer() {
  while(running) {
    {
      std::unique_lock<std::mutex> lock(mutex);
      while(!consume) {
        cv.wait(lock);
      }
    }
    while(consume) {
      std::this_thread::yield();
    }
  }
}

void term(int signum) {
  running = false;
}

int main(int argc, char** argv) {
  /** Attach the signal handler */
  struct sigaction action;
  memset(&action, 0, sizeof(struct sigaction));
  action.sa_handler = term;
  sigaction(SIGTERM, &action, NULL);

  /** Load the variables */
  const auto total_msec = 100U;
  float cpu_percentage = argc == 2 ? std::stof(argv[1]) : 50.0;
  running = true;
  consume = false;

  // Set the limits on the percentage
  cpu_percentage = std::min(100.0f, cpu_percentage);
  cpu_percentage = std::max(0.0f, cpu_percentage);
  std::cout << "Starting the consumer to use " << cpu_percentage << "% CPU" << std::endl;
  cpu_percentage = cpu_percentage / 100.0f;

  std::thread t(consumer);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Start the consumer
  while(running) {
    {
      std::unique_lock<std::mutex> lock(mutex);
      consume = true;
      cv.notify_all();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<uint32_t>(total_msec * cpu_percentage)));

    // Stop the consumer
    {
      std::unique_lock<std::mutex> lock(mutex);
      consume = false;
      cv.notify_all();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<uint32_t>(total_msec * (1.0f - cpu_percentage))));
  }

  std::cout << "Waiting for consumer to finish" << std::endl;
  t.join();

  return 0;
}
