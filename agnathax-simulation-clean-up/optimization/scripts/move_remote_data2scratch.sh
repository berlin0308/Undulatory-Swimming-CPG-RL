#!/bin/bash

for i in `seq 1 18`
do
	echo moving files from node $i
	ssh biorobcn$i 'mv /data/thandiac/test/log* /scratch/robin/.'
done