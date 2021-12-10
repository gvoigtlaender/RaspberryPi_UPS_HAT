#!/bin/sh

echo copying ups.service > /etc/systemd/system/
cp ups.service /etc/systemd/system/

echo ups.py /opt/
cp ups.py /opt/

echo installing python3-smbus 
apt install python3-smbus

echo enabling ups.service
systemctl enable ups.service

echo starting ups.service
systemctl start ups.service
