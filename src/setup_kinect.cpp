/*
 * This file is part of Moving Cells.
 *
 * Moving Cells is is a digital installation building on a depth sensor to
 * allow spectators to interact with a cloud of particles through their movements.
 * It has been developed and first displayed in June 2015 by Robin Lamarche-Perrin
 * and Bruno Pace for the eponymous dance festival, in Leipzig.
 * See: http://www.movingcells.org
 * 
 * The current version of the program is implemented on Kinect for Windows v2 (K4W2)
 * through the open source driver libreenect2.
 * See: https://github.com/OpenKinect/libfreenect2
 * 
 * Copyright © 2015-2017 Robin Lamarche-Perrin and Bruno Pace
 * (<Robin.Lamarche-Perrin@lip6.fr>)
 * 
 * Moving Cells is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 * 
 * Moving Cells is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <unistd.h>

#include "kinect.hpp"


int main (int argc, char *argv[])
{
	srand (time (NULL));

	Kinect *kinect = new Kinect ();
	kinect->graphicsWidth  = 1280;
	kinect->graphicsHeight = 720;

	kinect->xMin = -1.3;
	kinect->yMin =  1.3;
	kinect->zMin =  1.5;
	kinect->zMax =  2.5;
	kinect->rMin =  0.3;
	kinect->rMoy =  0.6;
	kinect->rMax =  1.0;

	kinect->weightMin = -0.5;
	kinect->weightMax =  1.0;

	kinect->thresholdFromFile = false;
	kinect->allowSensorDisplay = true;
	kinect->waitingTime = 10000;
	kinect->init();
	
	pthread_t kinectThread;
	int rcKinect = pthread_create (&kinectThread, NULL, &Kinect::run, (void *) kinect);
	if (rcKinect) { std::cout << "Error: Unable to create thread " << rcKinect << std::endl; exit (-1); }

	while (! kinect->stop) { usleep (30000); };

	void *status;
	rcKinect = pthread_join (kinectThread, &status);
	if (rcKinect) { std::cout << "Error: Unable to join thread " << rcKinect << std::endl; exit(-1); }

	delete kinect;
	
	return 0;
}

