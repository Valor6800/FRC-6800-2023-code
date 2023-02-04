#!/bin/bash

if [ -z "$1" ]; then
	echo "Please pass in a number 0-9 which is the last number in the robot 680X"
	exit 1
fi

if [ -d auto ];then
	mv auto auto.bak
fi

scp -r "lvuser@10.68.$1.2:~/auto" auto

if [ -d auto ];then
	rm -rf auto.bak
fi
