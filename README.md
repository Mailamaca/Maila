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

### Style

- for C++ .clang-format credits to DavetColemna [link](https://github.com/davetcoleman/roscpp_code_format/blob/master/.clang-format)

### VS-Code integration

- `f7` build all the packages

### Colcon commands

Build code

```bash
colcon build --symlink-install --cmake-args '-DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'
```

Run test

```bash
colcon test
colcon test-result
```

### Code formatting

https://answers.ros.org/question/325019/is-there-a-clang-format-file-for-ros-2/

https://github.com/ament/ament_lint/blob/26397786f603b8e9e4c3c399c3d33b1c6873ee0d/ament_clang_format/ament_clang_format/configuration/.clang-format

## Setup of Rasperry on the Vehicle

### install Ubuntu Server 20.04

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

## Setup VPN server
(ref: https://www.raspberrypi.org/forums/viewtopic.php?t=262264)
(ref: https://www.cactusvpn.com/tutorials/how-to-set-up-softether-vpn-client-on-linux/)
(ref: isc-dhcp-client)

- download softether vpn server 

```bash
cd ~
sudo apt update
sudo apt install -y cmake gcc g++ libncurses5-dev libreadline-dev libssl-dev make zlib1g-dev
git clone https://github.com/SoftEtherVPN/SoftEtherVPN_Stable.git
cd SoftEtherVPN_Stable
# ./configure # error during make "-m64 unrecognized", installing as 32bit
cp src/makefiles/linux_32bit.mak Makefile
make -j4
sudo make -j4 install
```

- create file for autoexecution at startup
(ref; https://linuxconfig.org/how-to-run-script-on-startup-on-ubuntu-20-04-focal-fossa-server-desktop)

```bash
sudo nano /etc/systemd/system/softether.service
```

with text:
```bash
[Unit]
After=network.service

[Service]
ExecStart=/usr/bin/vpnserver start
RemainAfterExit=yes

[Install]
WantedBy=default.target
```

- set as executable

```bash
sudo chmod 664 /etc/systemd/system/softether.service
```

- enable service

```bash
sudo systemctl daemon-reload
sudo systemctl enable softether.service
```

- settings of VPN server (via manager on windows)

```
vpn server maila-raspi
address: 192.168.25.60
https port: 5555
manager port: 443
admin user: maila
admin password: maila
IP-Sec preshred key: maila

users:
1:1
2:2
3:3
4:4
5:5
raspi:raspi
paoloto:paoloto
valerioma:valerioma
```

# Setup VPN client
(ref: https://www.cactusvpn.com/tutorials/how-to-set-up-softether-vpn-client-on-linux/)
(ref: https://serverfault.com/questions/586870/how-to-script-vpncmd-to-batch-command-connect-disconnect)

- run container

```bash
docker run --rm -it \
      --cap-add NET_ADMIN \
      --device /dev/net/tun \
      --name maila-dev-container -h maila \
      --user "$(id -u):$(id -g)" \
      --mount type=bind,src="$PWD",dst=/home/snail/Maila \
      -p 2222:22 \
      maila/maila-dev
```

- start vpn client batch file
(https://www.softether.org/4-docs/1-manual/6._Command_Line_Management_Utility_Manual/6.2_General_Usage_of_vpncmd#6.2.3_Command_Line_Parameters_When_Starting_a_vpncmd_Command)

```bash
nano vpnclient_batchstart.txt
```

with text:
```
NicCreate vpnnic
AccountCreate mailavpn /SERVER:192.168.25.60:443 /HUB:"DEFAULT" /USERNAME:"raspi" /NICNAME:"vpnnic"
AccountPassword mailavpn /PASSWORD="raspi" /TYPE="standard"
AccountConnect mailavpn
```

```bash
sudo vpnclient start
sudo vpncmd localhost /client /in:vpnclient_batchstart.txt
sudo /usr/sbin/dhclient
```

- test it!

```bash
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener
```

## run maila on docker image

- download maila repo 

```bash
cd ~
mkdir Projects
cd Projects
git clone https://github.com/Mailamaca/Maila.git
cd Maila
```

- pull image from DockerHub

```bash
docker pull maila/maila-dev
```

- run the container and start ssh

```bash
docker run --rm -it \
      --cap-add NET_ADMIN \
      --device /dev/net/tun \
      --mount type=bind,src=/home/ubuntu/SoftEtherVPN_Stable,dst=/home/snail/SoftEtherVPN_Stable \
      --name maila-dev-container -h maila \
      --user "$(id -u):$(id -g)" \
      --mount type=bind,src="$PWD",dst=/home/snail/Maila \
      -p 2222:22 \
      maila/maila-dev

sudo service ssh start

```

## run maila-dev on pc

```bash
docker run --rm -it \
      --cap-add NET_ADMIN \
      --device /dev/net/tun \
      --name maila-dev-container -h maila \
      --user "$(id -u):$(id -g)" \
      --mount type=bind,src="$PWD",dst=/home/snail/Maila \
      -p 2222:22 \
      maila/maila-dev

```

```bash
cd ~
sudo apt update
sudo apt install -y cmake gcc g++ libncurses5-dev libreadline-dev libssl-dev make zlib1g-dev
git clone https://github.com/SoftEtherVPN/SoftEtherVPN_Stable.git
cd SoftEtherVPN_Stable
./configure
#cp src/makefiles/linux_64bit.mak Makefile
make -j4
sudo make -j4 install
```

```
sudo apt install isc-dhcp-client
sudo /usr/sbin/dhclient
```
