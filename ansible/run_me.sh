#!/bin/bash
set -eu

if [[ $# -lt 1 ]]; then
  echo "usage: $0 command [args]"
  exit 1
fi

# Grab the latest ansible docker image
docker pull ansible/ansible-runner

# Start up the container and run the provisioning
docker run --rm -it -v $(pwd):/opt/ansible -v ~/.ssh:/root/.ssh --entrypoint $1 -w /opt/ansible ansible/ansible-runner:latest ${@:2}
