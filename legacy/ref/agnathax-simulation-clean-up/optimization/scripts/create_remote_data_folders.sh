#!/bin/bash

for i in `seq 1 18`
do
	echo creating folder on node $i
	ssh biorobcn$i 'mkdir /data/thandiac/test/'
done