#!/bin/bash
trash out/
mkdir out
../../build/bin/static-cells
trash out.HD/
mv out out.HD
trash static-cells-HD.mp4
ffmpeg -framerate 30 -i out.HD/static-cells-screenshot-%06d.png -c:v libx264 -profile:v high -crf 17 -pix_fmt yuv420p static-cells-HD.mp4
trash static-cells-alva-noto-HD.mp4
ffmpeg -i static-cells-HD.mp4 -itsoffset 0.70 -i alva-noto-outro-HD.mp3 -map 0:v -map 1:a -c copy -shortest static-cells-alva-noto-HD.mp4
./crop-video-HD.sh
#vlc static-cells-alva-noto-HD.mp4 &
