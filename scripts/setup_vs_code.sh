#!/bin/bash

if ! command -v code &> /dev/null
then
    echo "VScode could not be found installing it"
    # install repository and key 
    wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
    sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
    sudo sh -c 'echo "deb [arch=amd64 signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'

    # install vscode
    sudo apt-get install apt-transport-https
    sudo apt-get update
    sudo apt-get install code 
fi


echo "Installing VsCode Extensions"
# Docker 
EXTENSIONS=(
    "ms-vscode-remote.remote-containers"
    "ms-azuretools.vscode-docker"
)
for pkg in "${EXTENSIONS[@]}"
do
  echo "code --install-extension ${pkg}"
  code --install-extension ${pkg}
done


# Modern C++
EXTENSIONS=(
    "twxs.cmake"
    "ms-vscode.cpptools"
    "cschlosser.doxdocgen"
    "jbenden.c-cpp-flylint"
    "ms-vscode.cmake-tools"
    "cheshirekow.cmake-format"
    "llvm-vs-code-extensions.vscode-clangd"
)
for pkg in "${EXTENSIONS[@]}"
do
  echo "code --install-extension ${pkg}"
  code --install-extension ${pkg}
done

# Python
EXTENSIONS=(
  "ms-python.python"
  "ms-python.vscode-pylance"
)
for pkg in "${EXTENSIONS[@]}"
do
  echo "code --install-extension ${pkg}"
  code --install-extension ${pkg}
done

# Markdown
EXTENSIONS=(
    "yzhang.markdown-all-in-one"
    "DavidAnson.vscode-markdownlint"
    "shd101wyy.markdown-preview-enhanced"
)

for pkg in "${EXTENSIONS[@]}"
do
  echo "code --install-extension ${pkg}"
  code --install-extension ${pkg}
done

# Collaboration
EXTENSIONS=(
    "MS-vsliveshare.vsliveshare-pack"
    "karigari.chat"
)
for pkg in "${EXTENSIONS[@]}"
do
  echo "code --install-extension ${pkg}"
  code --install-extension ${pkg}
done

# Utility
EXTENSIONS=(
    "eamodio.gitlens"
    "dotjoshjohnson.xml"
    "vscode-icons-team.vscode-icons"
    "streetsidesoftware.code-spell-checker"
)
for pkg in "${EXTENSIONS[@]}"
do
  echo "code --install-extension ${pkg}"
  code --install-extension ${pkg}
done
