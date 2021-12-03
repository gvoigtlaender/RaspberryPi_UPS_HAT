#!/bin/sh

cp ups.service /etc/systemd/system/
cp ups.py /opt/
systemctrl enable ups.service
systemctrll start ups.service
