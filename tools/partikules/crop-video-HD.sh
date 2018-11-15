#!/bin/bash
trash out/
mkdir out
../../build/bin/static-cells
trash out.uncrop.HD/
mv out out.uncrop.HD
trash out.crop.HD/
mkdir out.crop.HD
for filename in ./out.uncrop.HD/*.png; do
	echo $(basename "$filename")
	./crop_screens/crop-screens ./out.uncrop.HD/$(basename "$filename") ./out.crop.HD/$(basename "$filename")
done
for crf in 15 20 25 30; do
	trash static-cells-crop-HD-$crf.mp4
	ffmpeg -framerate 30 -i out.crop.HD/static-cells-screenshot-%06d.png -c:v libx264 -profile:v high -crf $crf -pix_fmt yuv420p static-cells-crop-HD-$crf.mp4
	trash static-cells-alva-noto-crop-HD-$crf.mp4
	ffmpeg -i static-cells-crop-HD-$crf.mp4 -itsoffset 0.70 -i alva-noto-outro-HD.mp3 -map 0:v -map 1:a -c copy -shortest static-cells-alva-noto-crop-HD-$crf.mp4
done
#vlc static-cells-alva-noto-crop-HD.mp4 &
