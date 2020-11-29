#!/bin/bash

#########################################
# 			     Set paths 				          #
#########################################
SCRIPT_DIR=$( cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd)
WORKIN_DIR=${SCRIPT_DIR}/..

# Load maila environment
source ${WORKIN_DIR}/load_env.sh

#########################################
#   Install and config VScode   				#
#########################################
info "Setting up vscode"
${SCRIPT_DIR}/setup_vs_code.sh

#########################################
#   Create the docker container 				#
#########################################
print_delim
cd $SCRIPT_DIR
info "download maila docker image"
docker pull maila/maila-dev:latest

#########################################
#   Download needed repositories 				#
#########################################

print_delim
cd $SCRIPT_DIR
SCR_DIR="${WORKIN_DIR}/src"
if [ -d "${SCR_DIR}" ]; then
  ### Take action if $DIR exists ###
  info "Found src folder, updating required repositories in ${SCR_DIR}..."
else
  ###  Control will jump here if $DIR does NOT exists ###
  info "${SCR_DIR} not found, downloading required repositories..."
  mkdir ${WORKIN_DIR}/src  
fi

cd ${SCR_DIR}
vcs import ${SCR_DIR} < ${WORKIN_DIR}/.maila.repos

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


