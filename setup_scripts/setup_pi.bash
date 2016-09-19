#!/bin/bash

set -e

sudo apt-get update && sudo apt-get install -y python-dev python-pip libglib2.0-dev

echo "=== setup: motor driver"
sudo pip install wiringpi
git clone https://github.com/pololu/drv8835-motor-driver-rpi.git --depth 1
pushd drv8835-motor-driver-rpi
sudo python setup.py install
popd

echo "=== setup: sensortag"
git clone https://github.com/IanHarvey/bluepy.git --depth 1
pushd bluepy
python setup.py build && sudo python setup.py install
popd



