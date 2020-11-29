# This is the development environment
FROM maila/maila-dev:latest

# TODO: fix user permissions https://jtreminio.com/blog/running-docker-containers-as-current-host-user/
ARG USER_ID
ARG GROUP_ID

# create std user named snail
ENV USER_NAME snail
ENV GROUP_NAME snail

RUN groupadd -g ${GROUP_ID} ${GROUP_NAME} &&\
    useradd -l -u ${USER_ID} -g ${GROUP_NAME} ${USER_NAME} &&\
    install -d -m 0755 -o ${USER_NAME} -g ${GROUP_ID} /home/${USER_NAME} &&\
    usermod -aG sudo,video $USER_NAME && \
    echo "$USER_NAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USER_NAME && \
    chmod 0440 /etc/sudoers.d/$USER_NAME && \
    chown --changes --silent --no-dereference --recursive \
    --from=33:33 ${USER_ID}:${GROUP_ID} \
    /home/${USER_NAME}

# set user passwords
RUN echo 'root:root' |chpasswd
RUN echo "${USER_NAME}:${USER_NAME}" |chpasswd


# To enable gazebo gui:
#    -> easier (used) https://github.com/thewtex/docker-opengl-mesa
#    -> better alternative: https://github.com/thewtex/docker-opengl

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
  && apt-get -y --force-yes install x-window-system \
  # Clean up
  && apt-get autoremove -y \
  && apt-get clean -y \
  && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
  && apt-get -y install --no-install-recommends \
    mesa-utils \
  # Clean up
  && apt-get autoremove -y \
  && apt-get clean -y \
  && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

# Set working dir
USER ${USER_NAME}
RUN mkdir -p "/home/${USER_NAME}/ros_workspace"
WORKDIR "/home/${USER_NAME}/ros_workspace"

CMD ["bash"]
