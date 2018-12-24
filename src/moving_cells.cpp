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

#include "cloud.hpp"
#include "kinect.hpp"


int main (int argc, char *argv[])
{
	srand (time (NULL));

	Kinect *kinect = new Kinect ();
	kinect->graphicsWidth  = 1280;
	kinect->graphicsHeight = 720;

	kinect->xMin = -1.8;
	kinect->yMin =  1.8;
	kinect->zMin =  2.0;
	kinect->zMax =  4.2;
	kinect->rMin =  0.30;
	kinect->rMoy =  0.65;
	kinect->rMax =  0.95;

	kinect->weightMin = -0.5;
	kinect->weightMax =  1.0;

	kinect->thresholdFromFile = false;
	kinect->allowSensorDisplay = false;
	kinect->waitingTime = 100000;
	kinect->init();
	
	pthread_t kinectThread;
	int rcKinect = pthread_create (&kinectThread, NULL, &Kinect::run, (void *) kinect);
	if (rcKinect) { std::cout << "Error: Unable to create thread " << rcKinect << std::endl; exit (-1); }

	Cloud *cloud = new Cloud ();
	cloud->graphicsWidth  = 1280;
	cloud->graphicsHeight = 720;
	
	cloud->particleNumber = 1280 * 720 / 6;
	cloud->displayBodies  = false;
	cloud->particleDamping = 0.5;
	cloud->init();

	pthread_t cloudThread;
	int rcCloud = pthread_create (&cloudThread, NULL, &Cloud::run, (void *) cloud);
	if (rcCloud) { std::cout << "Error: Unable to create thread " << rcCloud << std::endl; exit (-1); }

	while (! cloud->stop && ! kinect->stop) {
		pthread_mutex_lock (&cloud->mutex);
		cloud->clearBodies();
		pthread_mutex_lock (&kinect->mutex);
		for (ObjectList::iterator it = kinect->objectList->begin(); it != kinect->objectList->end(); ++it) {
			Object *object = *it;
			cloud->addBody (new Body (object->x, object->y, object->weight));
		}
		pthread_mutex_unlock (&kinect->mutex);
		pthread_mutex_unlock (&cloud->mutex);
		usleep (30000);
	};

	if (! cloud->stop) {
		cloud->stop = true;
		void *status;
		rcCloud = pthread_join (cloudThread, &status);
		if (rcCloud) { std::cout << "Error: Unable to join thread " << rcCloud << std::endl; exit(-1); }
	}

	if (! kinect->stop) {
		kinect->stop = true;
		void *status;
		rcKinect = pthread_join (kinectThread, &status);
		if (rcKinect) { std::cout << "Error: Unable to join thread " << rcKinect << std::endl; exit(-1); }
	}
	
	// if (argc > 1) { cloud.configFilename = argv[1]; }
	// if (argc > 2) { cloud.outputFilename = argv[2]; }

	// Body *center = new Body ();
	// center->x = 0.5;
	// center->y = 0.25;
	// center->weight = 0.5;
	// //cloud.addBody (center);

	delete cloud;
	delete kinect;
	
	return 0;
}

