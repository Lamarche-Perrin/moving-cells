#!/bin/bash
trash out.crop.HD/
mkdir out.crop.HD
for filename in ./out.HD/*.png; do
	echo $(basename "$filename")
	./crop_screens/crop-screens ./out.HD/$(basename "$filename") ./out.crop.HD/$(basename "$filename")
done
trash static-cells-crop-HD.mp4
trash static-cells-alva-noto-crop-HD.mp4
ffmpeg -framerate 30 -i out.crop.HD/static-cells-screenshot-%06d.png -c:v libx264 -profile:v high -crf 30 -pix_fmt yuv420p static-cells-crop-HD.mp4
ffmpeg -i static-cells-crop-HD.mp4 -itsoffset 0.70 -i alva-noto-outro-HD.mp3 -map 0:v -map 1:a -c copy -shortest static-cells-alva-noto-crop-HD.mp4
#vlc static-cells-alva-noto-crop-HD.mp4 &
