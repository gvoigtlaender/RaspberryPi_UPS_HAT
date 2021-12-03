#!/usr/bin/python
import smbus
import time
import os
import syslog
import sys
from datetime import datetime
bus = smbus.SMBus(1)
address = 0x04
warn = 0
ncnt = 0

bus.write_byte_data(address, 4, 38)
bus.write_byte_data(address, 5, 45)


logfile = '/var/log/Raspi_UPS_HAT.log'
syslog.openlog(sys.argv[0])

def log(msg, bsyslog):
	print(msg)
	file = open(logfile,"a")
	file.write("%s: %s\n" % (time.strftime("%d.%m.%Y %H:%M:%S"), msg))
	file.close
	if bsyslog:
		syslog.syslog(msg)


while True:
	cap_min = 25
	bsyslog = ncnt % 1000 == 0
	ncnt =ncnt + 1
	now = datetime.now()
	dt_string = now.strftime("%d/%m/%Y %H:%M:%S")
	log("date and time ="+ dt_string, bsyslog)
	data = 0
	bus.write_byte(address, 0)
	voltage = bus.read_byte_data(address, 0)
	# print voltage
	capacity = bus.read_byte_data(address, 1)
	temperature = bus.read_byte_data(address, 2)
	status = bus.read_byte_data(address, 3)
	batmode = (status & 0x40 > 0);
	log("voltage %.1fV" % (voltage/10.0),bsyslog or batmode)
	log("capacity %.0f%%" % capacity,bsyslog or batmode);
	log("temperature: %.0fC" % temperature,bsyslog)
	log("button %d" % (status & 0x01 > 0),bsyslog)
	log("fan %d" % (status & 0x10 > 0),bsyslog)
	log("ac %d" % (status & 0x20 > 0),bsyslog)
	log("battery %d" % batmode,bsyslog)
	if capacity < cap_min and batmode == 1:
		warn = warn + 1
	if  warn == 1:
		log("shutting down", 1)
		time.sleep(1)
		os.system('shutdown -h now')
	time.sleep(1)
