#!/bin/bash

today=`date +'%Y_%m_%d'`
time=`date +'%H_%M_%S'`
dir=/mnt/SamsungKURI/B4_study/$today/kuriatsu

if [ -e $dir]; then
	echo "file found"
else
	mkdir -p $dir
fi

echo "record_start"
rosbag record -a -O $dir/$time.bag
