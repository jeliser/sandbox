#!/bin/bash

# Run the command, sleep, then terminate the process
while true; do
  ${@} &
  last_pid=$!
  sleep 5
  kill -9 $last_pid 2>/dev/null
done

