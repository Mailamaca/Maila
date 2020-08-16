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


updateRepo (){ 
    # from https://gist.github.com/nicferrier/2277987
    
    REPOSRC=$1
    LOCALREPO=$2
    BRANCH=$3

    FOLDER=$(pwd)
    
    # We do it this way so that we can abstract if from just git later on
    LOCALREPO_VC_DIR=$LOCALREPO/.git
    
    if [ ! -d $LOCALREPO_VC_DIR ]
    then
        info "${TAB}|-> Clonining ${REPOSRC}"
        git clone $REPOSRC $LOCALREPO -b $BRANCH
    else
        info "${TAB}|-> Pulling ${REPOSRC}"
        cd $LOCALREPO
        git pull origin $BRANCH
    fi
    cd $FOLDER
}


cd $SCRIPT_DIR
print_delim
info "Downloading required repositories"
mkcd "src"

updateRepo "https://github.com/Mailamaca/maila_msgs" "./maila_msgs" "master"
updateRepo "https://github.com/Mailamaca/maila_srvs" "./maila_srvs" "master"
updateRepo "https://github.com/Mailamaca/motors_interface" "./motors_interface" "master"
updateRepo "https://github.com/Mailamaca/joystick_drivers" "./joystick_drivers" "master"
updateRepo "https://github.com/Mailamaca/joystick_command" "./joystick_command" "master"
updateRepo "https://github.com/Mailamaca/gps_umd" "./gps_umd" "master"


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


