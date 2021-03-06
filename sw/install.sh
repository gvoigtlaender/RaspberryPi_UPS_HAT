#!/bin/sh

cd "$(dirname "$0")"
echo changed directory $(pwd)

echo "copying ups.service > /etc/systemd/system/"
cp ups.service /etc/systemd/system/ups.service

echo "ups.py /opt/"
cp ups.py /opt/ups.py

echo "installing python3-smbus"
apt-get -y install python3-smbus

echo "enabling ups.service"
systemctl enable ups.service

echo "starting ups.service"
systemctl start ups.service

echo "checking service status"
systemctl service ups.service
