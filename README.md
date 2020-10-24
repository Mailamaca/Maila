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

## Style

- for C++ .clang-format credits to DavetColemna [link](https://github.com/davetcoleman/roscpp_code_format/blob/master/.clang-format)


## VS-Code integration

- `f7` build all the packages


# Setup of Rasperry on the Vehicle

## install Ubuntu Server 20.04

(ref: https://itsfoss.com/install-ubuntu-server-raspberry-pi/)

- Write SD Card
  
- change file "system-boot/network-config" and add wifi SSID and password
  
```bash
wifis:
  wlan0:
    dhcp4: true
    optional: true
    access-points:
      "EssePi_24":
      password: "your_wifi_password"
```
- change hostname to "maila-raspi"
(ref: https://www.cyberciti.biz/faq/ubuntu-20-04-lts-change-hostname-permanently/)

```bash
sudo hostnamectl set-hostname maila-raspi
sudo reboot
```

- change password to "ubuntu:ubuntu"
(ref: https://www.cyberciti.biz/faq/change-a-user-password-in-ubuntu-linux-using-passwd/)

```bash
sudo passwd ubuntu
----
sudo reboot
```

- installing docker
(ref: https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-20-04)

```bash
sudo apt update
sudo dpkg --configure -a
sudo apt upgrade
sudo apt update
sudo apt install apt-transport-https ca-certificates curl software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository "deb [arch=$(dpkg --print-architecture)] https://download.docker.com/linux/ubuntu focal stable"
sudo apt update
apt-cache policy docker-ce
sudo apt install docker-ce
sudo systemctl status docker
sudo usermod -aG docker ${USER}
sudo reboot
```






