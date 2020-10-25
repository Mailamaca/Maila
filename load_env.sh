#!/usr/bin/env bash

# Export paths
MAILA_ROOT_DIR="$(cd "$( dirname "${BASH_SOURCE[0]}")" && pwd -P)"
MAILA_IN_DOCKER=false

# If inside docker container
if [ -f /.dockerenv ]; then
  MAILA_IN_DOCKER=true
fi

export MAILA_DOCKER_IMAGE="maila/maila-dev"
export MAILA_ROOT_DIR="${MAILA_ROOT_DIR}"
export MAILA_IN_DOCKER="${MAILA_IN_DOCKER}"

### Load bash utility and aliases
source ${MAILA_ROOT_DIR}/scripts/bash_utility
source ${MAILA_ROOT_DIR}/scripts/aliases

MAILA_logo


info "Sourcing maila/install/setup.bash\n"
# check whether devel folder exists
if [ -f "${MAILA_ROOT_DIR}/install/setup.bash" ]; then
    # source setup.sh from same directory as this file
    source "${MAILA_ROOT_DIR}/install/setup.bash"
else
    #source "/opt/ros/$ROS_DISTRO/setup.bash"
    warning "You need to build first before you can source\n ${TAB} Run 'MAILA_build' in the maila directory\n"
fi



