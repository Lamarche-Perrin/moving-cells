# Moving Cells

Moving Cells is a digital installation building on a depth sensor to allow spectators to interact with a cloud of particles through their movements. It has been developed and first displayed in June 2015 by Robin Lamarche-Perrin and Bruno Pace for the [eponymous dance festival](http://www.movingcells.org), in Leipzig.

The current version of the program is implemented on Kinect for Windows v2 (K4W2) through the open source driver [libreenect2](https://github.com/OpenKinect/libfreenect2). Note that this sensor also requires a USB 3.0 controller.

### Installation on Linux
* Install [libreenect2](https://github.com/OpenKinect/libfreenect2).
* Install OpenCV:
```
sudo apt-get install libopencv-dev
```

* Clone the project:
```
git clone https://github.com/Lamarche-Perrin/moving_cells
```

* Compile the files:
```
cd moving_cells
mkdir build
cd build
cmake ..
make
```

Note that you might need to adjust `CMAKE_PREFIX_PATH` in `CMakeLists.txt` to fit with the install location of the freenect2 library, that is something like `path/to/freenect2/lib/cmake/freenect2`.

* Run the programs:
```
./bin/moving-cells
./bin/time-delays
./bin/time-ghosts
./bin/webcam-delays
```


## License
Copyright © 2015-2017 Robin Lamarche-Perrin and Bruno Pace  
Contact: <Robin.Lamarche-Perrin@lip6.fr>

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see <http://www.gnu.org/licenses/>.
