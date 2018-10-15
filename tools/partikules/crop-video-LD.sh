#!/bin/bash
trash out.crop.LD/
mkdir out.crop.LD
for filename in ./out.LD/*.png; do
	echo $(basename "$filename")
	./crop_screens/crop-screens ./out.LD/$(basename "$filename") ./out.crop.LD/$(basename "$filename")
done
trash static-cells-crop-LD.mp4
trash static-cells-alva-noto-crop-LD.mp4
ffmpeg -framerate 30 -i out.crop.LD/static-cells-screenshot-%06d.png -c:v libx264 -profile:v high -crf 30 -pix_fmt yuv420p static-cells-crop-LD.mp4
ffmpeg -i static-cells-crop-LD.mp4 -itsoffset 0.70 -i alva-noto-outro-HD.mp3 -map 0:v -map 1:a -c copy -shortest static-cells-alva-noto-crop-LD.mp4
#vlc static-cells-alva-noto-crop-LD.mp4 &
