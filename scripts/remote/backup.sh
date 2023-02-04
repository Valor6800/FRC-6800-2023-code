#!/bin/bash

LOC_PROVISION="/home/lvuser/auto"

LOC_POINTS="$LOC_PROVISION/points"
LOC_AUTOS="$LOC_PROVISION/autos"
LOC_ACTIONS="$LOC_PROVISION/actions"

if [ ! -d $LOC_POINTS ] || [ ! -d $LOC_AUTOS ] || [ ! -d $LOC_ACTIONS ];then
	echo "Auto structure not found. Please initialize using the provision script"
	exit 1
fi

LOC_BAK_POINTS="$LOC_POINTS/backup"
LOC_BAK_AUTOS="$LOC_AUTOS/backup"
LOC_BAK_ACTIONS="$LOC_ACTIONS/backup"

if [ ! -d $LOC_BAK_POINTS ] || [ ! -d $LOC_BAK_AUTOS ] || [ ! -d $LOC_BAK_ACTIONS ];then
	mkdir -p "$LOC_BAK_POINTS"
	mkdir -p "$LOC_BAK_AUTOS"
	mkdir -p "$LOC_BAK_ACTIONS"
else
	diff --exclude=backup -ENwbu $LOC_ACTIONS $LOC_BAK_ACTIONS > "$LOC_BAK_ACTIONS.diff"
	diff --exclude=backup -ENwbu $LOC_AUTOS $LOC_BAK_AUTOS > "$LOC_BAK_AUTOS.diff"
	diff --exclude=backup -ENwbu $LOC_POINTS $LOC_BAK_POINTS > "$LOC_BAK_POINTS.diff"

	cat "$LOC_BAK_ACTIONS.diff"
	cat "$LOC_BAK_AUTOS.diff"
	cat "$LOC_BAK_POINTS.diff"
fi

cp $LOC_POINTS/*.csv $LOC_BAK_POINTS/.
cp $LOC_AUTOS/*.csv $LOC_BAK_AUTOS/.
cp $LOC_ACTIONS/*.csv $LOC_BAK_ACTIONS/.
