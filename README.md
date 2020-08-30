# Maila the snail (Mailamaca)
Main repository to setup the Maila project. The Maila project wants to build an autonomous car model on top of  a traxxas summit rc car

```bash
                o       o
                 \_____/
                 /=O=O=\     _______
                /   ^   \   /\\\\\\\\
                \ \___/ /  /\   ___  \
                 \_ V _/  /\   /\\\\  \
                   \  \__/\   /\ @_/  /
                    \____\____\______/
```

## Aliases 

- `MAILA_build` alias to build the entire project
- `MAILA_cd` navigate to the Maila root directory

## Requirements

- Docker. To intall Docker follow the procedure reported [here](https://docs.docker.com/engine/install/ubuntu/)
- Vcstool is a version control system (VCS) tool, designed to make working with multiple repositories easier. [link](https://github.com/dirk-thomas/vcstool)

```bash
sudo pip install vcstool
```

or

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xAB17C654
sudo apt-get update
sudo apt-get install python3-vcstool
```



## Setup

Download the docker image and the project scripts

```bash
git clone https://github.com/Mailamaca/Maila.git
cd Maila
./initialize_project.sh
```

If you want to automatically source the aliases at the end of `.bashrc` file add the follwing lines

```bash
source <MAILA_DIR>/scripts/load_environment.sh
```