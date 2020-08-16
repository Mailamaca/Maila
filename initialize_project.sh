#!/bin/bash

#########################################
# 			     Set paths 				          #
#########################################
SCRIPT_DIR=$( cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd)
source ${SCRIPT_DIR}/scripts/load_environment.sh

print_delim
printf "\n"
printf "Maila workspace initializer script\n"
printf "Version 0.1 - 2020"
printf "\n\n"

#########################################
#   Create the docker container 				#
#########################################
cd $SCRIPT_DIR
print_delim
info "Setting up docker"

# Clone the repository
info "Cloning docker repository"
git clone https://github.com/Mailamaca/Maila_docker.git .docker -b master

cd .docker

info "Building maila docker image"
docker build -t maila-image .

#########################################
#   Download needed repositories 				#
#########################################
cd $SCRIPT_DIR
print_delim
info "Downloading required repositories"
mkcd "src"

info "${TAB}|-> 01) Maila_ROSNode_MotorsInterface"
git clone https://github.com/Mailamaca/Maila_ROSNode_MotorsInterface.git -b master

#########################################
#   Final message               				#
#########################################
cd $SCRIPT_DIR
print_delim
ok "Setup terminated"
print_delim
info "Run 'd_run' to run the docker container"
info "Run 'd_bash' to open a new terminal in the running container"
info "Run 'MAILA_build' inside the container to compile the code"
print_delim


