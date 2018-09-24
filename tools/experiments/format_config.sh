#!/bin/bash
for f in config/done/*;
do
	#tail -n +2 "$f" > "$f.tmp" && mv "$f.tmp" "$f"
done
