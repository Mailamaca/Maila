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

### Docker

- `d_bash`  Connect current terminal to a running docker container
- `d_run` Run the maila docker container

### Folder navigation:

- `MAILA_cd` = `cd <maila>/`
- `MAILA_cds` = `cd <maila>/src` 
- `MAILA_cdd` = `cd <maila>/devel`
- `MAILA_cdi` = `cd <maila>/install`

### Code compilation

- `MAILA_build` build the code
- `MAILA_force_clean` clean the project

### Miscellanea

- `MAILA_source` source the MAILA environment

## Setup the workspace

### Requirements

- Docker. To intall Docker follow the procedure reported [here](https://docs.docker.com/engine/install/ubuntu/)

- Vcstool is a version control system (VCS) tool, designed to make working with multiple repositories easier. [link](https://github.com/dirk-thomas/vcstool)

  ```shell
  sudo pip install vcstool
  ```

- Install [Visual Studio Code](https://code.visualstudio.com/) 

### First Setup

There is a handy installation script that clones the repository list contained in `.maila.repos`, install the necessary *Visual Studio Code* extension and load the environment. If an `src` folder already exist in the repository the clone step is skipped.

```bash
cd scripts/
./init.sh	
```

### Load environment

To load the project aliases, the ros packages and environment variables run:

```shell
. load_env.sh
```

If you want to automatically source the maila environment add the step on`~/.bashrc` .



## RANDOM STUFF TO CLEAN

## Style

- for C++ .clang-format credits to DavetColemna [link](https://github.com/davetcoleman/roscpp_code_format/blob/master/.clang-format)


## VS-Code integration

- `f7` build all the packages

## Colcon commands
Build code
```bash
colcon build --symlink-install --cmake-args '-DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'
```

Run test

```bash
colcon test
colcon test-result
```

## Code formatting
https://answers.ros.org/question/325019/is-there-a-clang-format-file-for-ros-2/

https://github.com/ament/ament_lint/blob/26397786f603b8e9e4c3c399c3d33b1c6873ee0d/ament_clang_format/ament_clang_format/configuration/.clang-format