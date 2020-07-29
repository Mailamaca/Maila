#!/usr/bin/env bash

# Define style for print
BOLD='\033[1m'
RED='\033[0;31m'
BLUE='\033[0;34m'
GREEN='\033[32m'
WHITE='\033[34m'
YELLOW='\033[33m'
NO_COLOR='\033[0m'
export TAB="    " # 4 spaces


# Export paths
MAILA_ROOT_DIR="$(cd "$( dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
MAILA_IN_DOCKER=false

# If inside docker container
if [ -f /.dockerenv ]; then
  MAILA_IN_DOCKER=true
  MAILA_ROOT_DIR="/workspace"
fi

export MAILA_ROOT_DIR="${MAILA_ROOT_DIR}"
export MAILA_IN_DOCKER="${MAILA_IN_DOCKER}"

### Load bash utility and aliases
info "Sourcing bash aliases"
source ${MAILA_ROOT_DIR}/scripts/bash_utility
source ${MAILA_ROOT_DIR}/scripts/aliases

info "Sourcing maila/install/setup.bash\n"
# check whether devel folder exists
if [ -f "${MAILA_ROOT_DIR}/install/setup.bash" ]; then
    # source setup.sh from same directory as this file
    source "${MAILA_ROOT_DIR}/install/setup.bash"
else
    #source "/opt/ros/$ROS_DISTRO/setup.bash"
    warning "You need to build first before you can source\n ${TAB} Run 'colcon build' in the maila directory\n"
fi



