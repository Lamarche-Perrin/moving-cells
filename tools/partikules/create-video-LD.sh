#!/bin/bash
trash out/
mkdir out
../../build/bin/static-cells
trash out.LD/
mv out out.LD
trash static-cells-LD.mp4
trash static-cells-alva-noto-LD.mp4
ffmpeg -framerate 30 -i out.LD/static-cells-screenshot-%06d.png -c:v libx264 -profile:v high -crf 30 -pix_fmt yuv420p static-cells-LD.mp4
ffmpeg -i static-cells-LD.mp4 -itsoffset 0.70 -i alva-noto-outro-HD.mp3 -map 0:v -map 1:a -c copy -shortest static-cells-alva-noto-LD.mp4
./crop-video-LD.sh
#vlc static-cells-alva-noto-LD.mp4 &
