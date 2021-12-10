#!/bin/sh

cp ups.service /etc/systemd/system/
cp ups.py /opt/
systemctl enable ups.service
systemctl start ups.service
