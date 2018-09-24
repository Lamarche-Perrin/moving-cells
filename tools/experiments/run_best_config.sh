#!/bin/bash
cat fitness.csv | while IFS=, read -r num fitness
do
    if [ $fitness -ge 4 ] && [ $num -ge 47 ]
	then
		strnum=$(printf %06d $num)
		filename="random-config-$strnum";
		echo "running $filename"
		../../build/bin/static-cells config/done/$filename output/$filename
	fi
done
