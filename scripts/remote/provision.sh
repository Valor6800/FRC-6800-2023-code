#!/bin/bash

LOC_PROVISION="/home/lvuser/auto"

mkdir -p "$LOC_PROVISION"
mv ~/backup.sh $LOC_PROVISION/.

LOC_POINTS="$LOC_PROVISION/points"
LOC_AUTOS="$LOC_PROVISION/autos"
LOC_ACTIONS="$LOC_PROVISION/actions"

mkdir -p "$LOC_POINTS"
mkdir -p "$LOC_ACTIONS"
mkdir -p "$LOC_AUTOS"

FILE_POINTS_RED="$LOC_POINTS/points_red.csv"
FILE_POINTS_BLUE="$LOC_POINTS/points_blue.csv"

cat <<EOT > "$FILE_POINTS_RED"
start,0,0
EOT
cp "$FILE_POINTS_RED" "$FILE_POINTS_BLUE"

FILE_ACTIONS="$LOC_ACTIONS/wait_1sec.csv"

cat <<EOT > "$FILE_ACTIONS"
time,1000
EOT

FILE_AUTO_RED="$LOC_AUTOS/auto_default_red.csv"
FILE_AUTO_BLUE="$LOC_AUTOS/auto_default_blue.csv"

cat <<EOT > "$FILE_AUTO_RED"
action,wait_1sec
EOT
cp "$FILE_AUTO_RED" "$FILE_AUTO_BLUE"

$LOC_PROVISION/backup.sh "$LOC_PROVISION"
