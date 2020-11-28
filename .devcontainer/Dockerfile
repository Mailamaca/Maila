# This is the development environment
FROM maila/maila-dev:latest

# ** [Optional] Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
  && apt-get -y install --no-install-recommends \
    mesa-utils \
  # Clean up
  && apt-get autoremove -y \
  && apt-get clean -y \
  && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
  && apt-get -y install --force-yes x-window-system \
  # Clean up
  && apt-get autoremove -y \
  && apt-get clean -y \
  && rm -rf /var/lib/apt/lists/*
  
# create std user named snail
RUN useradd --create-home -ms /bin/bash snail
RUN usermod -aG sudo snail

# set user passwords
RUN echo 'root:root' |chpasswd
RUN echo 'snail:snail' |chpasswd

# Set working dir
RUN mkdir -p /home/snail/ros_workspace
WORKDIR /home/snail/ros_workspace
USER snail

CMD ["bash"]
