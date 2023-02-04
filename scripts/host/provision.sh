#!/bin/bash

if [ -z "$1" ]; then
	echo "Please pass in a number 0-9 which is the last number in the robot 680X"
	exit 1
fi

scp ./remote/backup.sh "lvuser@10.68.$1.2:~/backup.sh"

ssh "lvuser@10.68.$1.2" 'bash -s' < ./remote/provision.sh
