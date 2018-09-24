#!/bin/bash
for f in config/*;
do
	filename=`basename $f`;
	echo "running $filename"
	../../build/bin/static-cells config/$filename output/$filename
done
