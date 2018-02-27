#!/bin/bash

echo "We're going to install and setup Docker"

sudo apt-get update
sudo apt-get install apt-transport-https ca-certificates curl \
    software-properties-common

# Add the docker GPG key
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -

sudo apt-key fingerprint 0EBFCD88
echo "Verify that the key matches the following:"
echo "9DC8 5822 9FC7 DD38 854A E2D8 8D81 803C 0EBF CD88"

read -p "Press Enter when verified"

sudo add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
   $(lsb_release -cs) \
   stable"
sudo apt-get update
sudo apt-get install docker-ce

