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
info "download maila docker image"
docker pull maila/maila_dev:latest

#########################################
#   Download needed repositories 				#
#########################################

cd $SCRIPT_DIR
print_delim
info "Downloading required repositories"
mkcd "src"
vcs import $SCRIPT_DIR/src < $SCRIPT_DIR/maila.repos

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


