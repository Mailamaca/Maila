#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"


d_attach(){ # Attach to a running container
  CONTAINER=$(docker ps | grep "\w*vsc-maila\S*" | awk '{print $NF}')
  echo "Running INSIDE docker $CONTAINER"
  docker exec -it $CONTAINER bash
  # TO CONNECT AS ROOT
  #docker exec -u 0 -it $CONTAINER bash  
}

CONTAINER=$(docker ps | grep "\w*maila-container\S*" | awk '{print $NF}')
if [ -z "$CONTAINER" ]
then
      # If no running container is found
      echo "Running OUTSIDE docker"
      echo "run code ./ or run d_run to start the docker container "

      DOCKER_REMOTE_IMAGE="maila/maila-dev:latest"
      DOCKER_LOCAL_IMAGE="maila-dev-vscode"

      # Pull the remote_base_image
      d_pull_remote(){
            docker pull ${DOCKER_REMOTE_IMAGE}
      }

      # Build the container
      d_build(){
            cd $DIR/.devcontainer
            docker build . -f Dockerfile -t ${DOCKER_LOCAL_IMAGE}  --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g)
            cd $DIR
      }

      # Run the docker container
      d_run () {
        # Settings for gazebo GUI
        XSOCK=/tmp/.X11-unix
        XAUTH=/tmp/.docker.xauth
        touch $XAUTH
        xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
        docker run --rm -it \
            --name manual-maila-container -h snail \
            --mount type=bind,src=$DIR,dst=/home/snail/ros_workspace \
            --mount type=bind,src=${HOME}/.bash_aliases,dst=/home/snail/.bash_aliases \
            --env DISPLAY=$DISPLAY \
            --network=host \
            --cap-add=SYS_PTRACE \
            --security-opt=seccomp:unconfined  \
            --security-opt=apparmor:unconfined \
            -v $XSOCK:$XSOCK:rw \
            -v $XAUTH:$XAUTH:rw \
            --device=/dev/dri/card0:/dev/dri/card0 \
            -e XAUTHORITY=$XAUTH \
            ${DOCKER_LOCAL_IMAGE}
      }

else
  # If running container is found
  d_attach
fi
