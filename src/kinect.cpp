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
 * Copyright © 2015-2018 Robin Lamarche-Perrin and Bruno Pace
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

#include <iostream>
#include <cstdio>
#include <signal.h>
#include <fstream>
#include <sys/time.h>
#include <errno.h>
#include <unistd.h>

#include "kinect.hpp"


// int main (int argc, char *argv[])
// {
// 	Kinect kinect;
// 	kinect.run();
// }


Kinect::Kinect () {}

void Kinect::init ()
{
	// std::string program_path (argv[0]);
	// size_t executable_name_idx = program_path.rfind ("moving-cells");

	// std::string binpath = "/";
	// if (executable_name_idx != std::string::npos) { binpath = program_path.substr (0, executable_name_idx); }

	if (freenect2.enumerateDevices() == 0) { std::cout << "no device connected!" << std::endl; return; }

	std::string serial = freenect2.getDefaultDeviceSerialNumber();
	pipeline = new libfreenect2::OpenGLPacketPipeline();
	dev = freenect2.openDevice (serial, pipeline);

	bool viewer_enabled = true;

	if (dev == 0) { std::cout << "failure opening device!" << std::endl; return; }

	//signal (SIGINT, sigint_handler);
	stop = false;

	listener = new libfreenect2::SyncMultiFrameListener (libfreenect2::Frame::Depth);
	//libfreenect2::SyncMultiFrameListener listener (libfreenect2::Frame::Color | libfreenect2::Frame::Depth);

	dev->setIrAndDepthFrameListener (listener);
	dev->start();

	std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
	std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
	registration = new libfreenect2::Registration (dev->getIrCameraParams(), dev->getColorCameraParams());
	
	size_t framecount = 0;
	
	// if (allowKinectCalibration)
	// {
	// 	std::ofstream file;
	// 	file.open ("calibrate/calibrate.input", std::ios::out | std::ios::trunc);
	// 	file << "X;Y;Z;AXE;TYPE;V1;V2" << std::endl;
	// 	file.close();
	// }

	objectCounter = 0;
	thresholdFrame = 0;
    objectList = new ObjectList();

	struct timeval kinectStartTimer, kinectEndTimer;
	gettimeofday (&kinectStartTimer, NULL);

	int kinectFrameCounter = 0;
	double kinectDelay = 0;
	double kinectSumDelay = 0;

	stop = false;

	if (thresholdFromFile)
	{
		thresholdFrame = new cv::Mat();
		cv::FileStorage file ("threshold.ext", cv::FileStorage::READ);
		file["threshold"] >> (*thresholdFrame);
		file.release();
		thresholdKinect = false;
	}
	else { thresholdKinect = true; }
}


Kinect::~Kinect ()
{
	dev->stop();
	dev->close();
}


	// if (allowGraphicsDisplay)
	// {
	// 	int rc = pthread_create (&loopThread, NULL, loop, NULL);
	// 	if (rc) { std::cout << "Error:unable to create thread," << rc << std::endl; exit(-1); }
	// }


void *Kinect::run (void *arg)
{
	reinterpret_cast<Kinect*>(arg)->run();
	pthread_exit (NULL);
}


void Kinect::run ()
{
	while (! stop)
	{
		// COMPUTE TIME
		gettimeofday (&kinectEndTimer, NULL);
		kinectDelay = (kinectEndTimer.tv_sec - kinectStartTimer.tv_sec) + (float)(kinectEndTimer.tv_usec - kinectStartTimer.tv_usec) / 1000000L;
		kinectStartTimer = kinectEndTimer;

		kinectFrameCounter++;
		kinectSumDelay += kinectDelay;

		if (kinectSumDelay >= 3)
		{
			kinectFps = (int) (((float) kinectFrameCounter) / kinectSumDelay);
			std::cout << "KINECT: " << kinectFps << "fps" << std::endl;
			kinectSumDelay = 0;
			kinectFrameCounter = 0;
		}	

		// GET NEW FRAME
		listener->waitForNewFrame (frames);
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
		cv::Mat *depthFrame = new cv::Mat (depth->height, depth->width, CV_32FC1, depth->data);

		libfreenect2::Frame *undepth;
		if (realPositioning) {
			undepth = new libfreenect2::Frame (depthWidth, depthHeight, 4);
			registration->undistortDepth (depth, undepth);
			//cv::Mat *undepthFrame = new cv::Mat (undepth->height, undepth->width, CV_32FC1, undepth->data);
			//cv::imshow ("undepth", *undepthFrame / 4500.);
		}


		// THRESHOLD KINECT IF REQUIRED
		if (thresholdKinect)
		{
			std::cout << "THRESHOLDING...";
			
			if (thresholdFrame != 0) { delete thresholdFrame; }
			thresholdFrame = new cv::Mat(depth->height, depth->width, CV_32FC1, double(0));
			for (int x = 0; x < depth->width; x++)
				for (int y = 0; y < depth->height; y++)
					if (depthFrame->at<float>(cv::Point(x,y)) > 0)
					{ thresholdFrame->at<float>(cv::Point(x,y)) = depthFrame->at<float>(cv::Point(x,y)) - thresholdAdd; }
					else { thresholdFrame->at<float>(cv::Point(x,y)) = 0; }

			thresholdKinect = false;
			std::cout << " DONE" << std::endl;

			cv::FileStorage file ("threshold.ext", cv::FileStorage::WRITE);
			file << "threshold" << *thresholdFrame;
			file.release();
		}

		// APPLY THRESHOLD TO CURRENT FRAME
		float *dPixel = depthFrame->ptr<float>(0);
		float *tPixel = thresholdFrame->ptr<float>(0);
		for (int i = 0; i < depth->width*depth->height; i++)
			if ((tPixel[i] != 0 && dPixel[i] > tPixel[i]) || (distanceMax != 0 && dPixel[i] > distanceMax)) { dPixel[i] = 0; }


		// DETECT OBJECTS
		newObjectList = new ObjectList();
		if (realPositioning) { extractObjects (dPixel, undepth); } else { extractObjects (dPixel); }

		//for (ObjectList::iterator it = objectList->begin(); it != objectList->end(); ++it) { (*it)->getClosestObject (newObjectList); }
		for (ObjectList::iterator it = newObjectList->begin(); it != newObjectList->end(); ++it) { (*it)->update (kinectDelay); }
		computeObjects ();

		// std::cout << "NEW OBJECT LIST" << std::endl;
		// for (ObjectList::iterator it = newObjectList->begin(); it != newObjectList->end(); ++it) { (*it)->print(); }

		pthread_mutex_lock (&mutex);
		for (ObjectList::iterator it = objectList->begin(); it != objectList->end(); ++it) { delete (*it); }
		delete objectList;
		objectList = newObjectList;
		pthread_mutex_unlock (&mutex);

		// std::cout << "    OBJECT LIST" << std::endl;
		// for (ObjectList::iterator it = objectList->begin(); it != objectList->end(); ++it) { (*it)->print(); }
				
		// pthread_mutex_lock(&mutex);
		// if (currentObjectList != objectList)
		// {
		// 	for (ObjectList::iterator it = currentObjectList->begin(); it != currentObjectList->end(); ++it) { delete (*it); }
		// 	delete currentObjectList;
		// }
		// currentObjectList = objectList;
		// pthread_mutex_unlock(&mutex);


		// DISPLAY SENSOR
		if (allowSensorDisplay) { displaySensor(depthFrame); }

		int key = cv::waitKey (1);
		if (key > 0)
		{
			key = key & 0xFF;
			std::cout << "KINECT KEY PRESSED: " << key << std::endl;

			stop = stop || key == 27; // Escape
			thresholdKinect = thresholdKinect || key == 8; // Backspace

			if (allowKinectCalibration) { calibrateKinect(key); }
		}

		listener->release (frames);
		delete depthFrame;
		//libfreenect2::this_thread::sleep_for(libfreenect2::chrono::milliseconds(100));

		usleep (waitingTime);
	}
}




void Kinect::sigint_handler (int s) { }

int Kinect::scale (float z) { return round((z-500)/10); }


void Object::getClosestObject (ObjectList *list) {
	this->closestObject = 0;
	this->minDist = -1;
	for (ObjectList::iterator it = list->begin(); it != list->end(); ++it)
	{
		Object *newObject = *it;
		double dist = this->getDistance(newObject);
		if (newObject->minDist < 0 || newObject->minDist > dist)
		{
			if (this->closestObject == 0 || dist < minDist)
			{
				this->closestObject = newObject;
				this->minDist = dist;
			}
		}
	}
	
	if (this->closestObject != 0)
	{
		Object *oldObject = this->closestObject->closestObject;
		this->closestObject->closestObject = this;
		this->closestObject->minDist = this->minDist;
		if (oldObject != 0) { oldObject->getClosestObject(list); }
	}
}


void Object::update (double delay) {
	if (this->closestObject == 0)
	{
		this->index = kinect->objectCounter++;
		xMinS = 0; xMoyS = 0; xMaxS = 0; yMinS = 0; yMoyS = 0; yMaxS = 0; zMinS = 0; zMoyS = 0; zMaxS = 0;
	}

	else {
		Object *b = this->closestObject;
		this->index = b->index;
		xMinS = ((float)(xMin - b->xMin)) / delay;
		xMoyS = ((float)(xMoy - b->xMoy)) / delay;
		xMaxS = ((float)(xMax - b->xMax)) / delay;
		yMinS = ((float)(yMin - b->yMin)) / delay;
		yMoyS = ((float)(yMoy - b->yMoy)) / delay;
		yMaxS = ((float)(yMax - b->yMax)) / delay;
		zMinS = ((float)(zMin - b->zMin)) / delay;
		zMoyS = ((float)(zMoy - b->zMoy)) / delay;
		zMaxS = ((float)(zMax - b->zMax)) / delay;
	}
	this->closestObject = 0;
	this->minDist = -1;
}


double Object::getDistance (Object *object) {
	return sqrt( pow(this->xMoy-object->xMoy,2) + pow(this->yMoy-object->yMoy,2) + pow(this->zMoy-object->zMoy,2));
}


void Object::print ()
{
	std::cout << "OBJECT " << index << " (" << rxMin << "," << rxMoy << "," << rxMax << ") "
			  << "(" << ryMin << "," << ryMoy << "," << ryMax << ") "
			  << "(" << rzMin << "," << rzMoy << "," << rzMax << ")" << std::endl;
	std::cout << " -> (" << x << ", " << y << ") = " << weight << std::endl;
}



void Kinect::extractObjects (float *dPixel)
{
	cv::Mat *foundFrame = new cv::Mat (depthHeight, depthWidth, CV_32FC1, double(0));
	float *fPixel = foundFrame->ptr<float>(0);
	for (int i = 0; i < depthWidth*depthHeight; i++)
	{
		if (dPixel[i] > 0 && fPixel[i] == 0)
		{
			int x = i % depthWidth;
			int y = (i - x) / depthWidth;
			int z = scale(dPixel[i]);	    

			Object *object = new Object (this);
			object->pixelNb = 0;
			object->xMin = x;
			object->xMoy = 0;
			object->xMax = x;
			object->yMin = y;
			object->yMoy = 0;
			object->yMax = y;
			object->zMin = z;
			object->zMoy = 0;
			object->zMax = z;
			object->closestObject = 0;
			object->minDist = -1;

			std::list<Pixel> pixelList;
			pixelList.push_back(Pixel(x,y,z));
			foundFrame->at<float>(cv::Point(x,y)) = 1;
		    
			while (!pixelList.empty())
			{
				Pixel p = pixelList.front();
				pixelList.pop_front();

				object->pixelNb++;
				object->xMoy += p.x;
				object->yMoy += p.y;
				object->zMoy += p.z;

				if (p.x < object->xMin) { object->xMin = p.x; }
				if (p.x > object->xMax) { object->xMax = p.x; }
				if (p.y < object->yMin) { object->yMin = p.y; }
				if (p.y > object->yMax) { object->yMax = p.y; }
				if (p.z < object->zMin) { object->zMin = p.z; }
				if (p.z > object->zMax) { object->zMax = p.z; }

				int s = 1;
				for (int dx = -s; dx <= s; dx++)
					for (int dy = -s; dy <= s; dy++)
					{
						if (dx !=0 && dy !=0) { continue; }
						int nx = p.x+dx;
						int ny = p.y+dy;
						int ni = nx+ny*depthWidth;
						if (nx >=0 && nx < depthWidth && ny >= 0 && ny < depthHeight)
							if (dPixel[ni] > 0 && fPixel[ni] == 0)
							{
								int nz = scale(dPixel[ni]);
								pixelList.push_back(Pixel(nx,ny,nz));
								fPixel[ni] = 1;
							}
					}
			}

			object->xMoy /= object->pixelNb;
			object->yMoy /= object->pixelNb;
			object->zMoy /= object->pixelNb;
			
			object->ratio = (object->xMax - object->xMin) / (object->yMax - object->yMin);
			
			if (object->pixelNb >= objectMinSize) { newObjectList->push_back(object); }
			else { delete object; }
		}
	}
	delete foundFrame;
}



void Kinect::extractObjects (float *dPixel, libfreenect2::Frame *undepth)
{
	cv::Mat *foundFrame = new cv::Mat (depthHeight, depthWidth, CV_32FC1, double(0));
	float *fPixel = foundFrame->ptr<float>(0);
	for (int y = 0; y < depthHeight; y++)
	{
		int i = depthWidth * y;
		for (int x = 0; x < depthWidth; x++)
		{
			if (dPixel[i] > 0 && fPixel[i] == 0)
			{	
				int z = scale(dPixel[i]);

				float rx, ry, rz;
				registration->getPointXYZ (undepth, y, x, rx, ry, rz);

				Object *object = new Object (this);
				object->closestObject = 0;
				object->minDist = -1;

				object->pixelNb = 0;
				object->extrema = false;
				object->rextrema = false;

				if (realMoy) {
					object->rpixelNb = 0;		
					object->xMoy = 0;
					object->yMoy = 0;
					object->zMoy = 0;
					object->rxMoy = 0;
					object->ryMoy = 0;
					object->rzMoy = 0;
				}
				
				std::list<Pixel> pixelList;
				pixelList.push_back(Pixel(x,y,z,rx,ry,rz));
				foundFrame->at<float>(cv::Point(x,y)) = 1;
		    
				while (!pixelList.empty())
				{
					Pixel p = pixelList.front();
					pixelList.pop_front();

					object->pixelNb++;
					if (realMoy) {
						object->xMoy += p.x;
						object->yMoy += p.y;
						object->zMoy += p.z;
					}

					if (! object->extrema) {
						object->xMin = x;
						object->xMax = x;
						object->yMin = y;
						object->yMax = y;
						object->zMin = z;
						object->zMax = z;
						object->extrema = true;
					}

					else {
						if (p.x < object->xMin) { object->xMin = p.x; }
						if (p.x > object->xMax) { object->xMax = p.x; }
						if (p.y < object->yMin) { object->yMin = p.y; }
						if (p.y > object->yMax) { object->yMax = p.y; }
						if (p.z < object->zMin) { object->zMin = p.z; }
						if (p.z > object->zMax) { object->zMax = p.z; }
					}

					if (p.rx == p.rx && p.ry == p.ry && p.rz == p.rz) {
						if (realMoy) {
							object->rpixelNb++;					
							object->rxMoy += p.rx;
							object->ryMoy += p.ry;
							object->rzMoy += p.rz;
						}

						if (! object->rextrema) {
							object->rxMin = p.rx;
							object->rxMax = p.rx;
							object->ryMin = p.ry;
							object->ryMax = p.ry;
							object->rzMin = p.rz;
							object->rzMax = p.rz;
							object->rextrema = true;
						}

						else {
							if (p.rx < object->rxMin) { object->rxMin = p.rx; }
							if (p.rx > object->rxMax) { object->rxMax = p.rx; }
							if (p.ry < object->ryMin) { object->ryMin = p.ry; }
							if (p.ry > object->ryMax) { object->ryMax = p.ry; }
							if (p.rz < object->rzMin) { object->rzMin = p.rz; }
							if (p.rz > object->rzMax) { object->rzMax = p.rz; }
						}
					}


					int s = 1;
					for (int dx = -s; dx <= s; dx++)
						for (int dy = -s; dy <= s; dy++)
						{
							if (dx !=0 && dy !=0) { continue; }
							int nx = p.x+dx;
							int ny = p.y+dy;
							int ni = nx+ny*depthWidth;
							if (nx >= 0 && nx < depthWidth && ny >= 0 && ny < depthHeight)
								//if (nx >= depthCropLeft && nx < (depthWidth - depthCropRight) && ny >= depthCropTop && ny < (depthHeight -depthCropBottom))
								if (dPixel[ni] > 0 && fPixel[ni] == 0)
								{
									int nz = scale(dPixel[ni]);
									float nrx, nry, nrz;
									registration->getPointXYZ (undepth, ny, nx, nrx, nry, nrz);

									if (abs (nrz - p.rz) < 0.1) {
										pixelList.push_back(Pixel(nx,ny,nz,nrx,nry,nrz));
										fPixel[ni] = 1;
									}
								}
						}
				}

				if (object->pixelNb >= objectMinSize && (!realMoy || object->rpixelNb >= objectMinSize)) {
					if (realMoy) {
						object->xMoy /= object->pixelNb;
						object->yMoy /= object->pixelNb;
						object->zMoy /= object->pixelNb;

						object->rxMoy /= object->rpixelNb;
						object->ryMoy /= object->rpixelNb;
						object->rzMoy /= object->rpixelNb;
					}

					else {
						object->xMoy = (object->xMax + object->xMin) / 2;
						object->yMoy = (object->yMax + object->yMin) / 2;
						object->zMoy = (object->zMax + object->zMin) / 2;

						object->rxMoy = (object->rxMax + object->rxMin) / 2;
						object->ryMoy = (object->ryMax + object->ryMin) / 2;
						object->rzMoy = (object->rzMax + object->rzMin) / 2;
					}

					object->ratio = (object->rxMax - object->rxMin) / (object->ryMax - object->ryMin);

					if (object->xMoy < depthCropLeft || object->xMoy >= (depthWidth - depthCropRight) || object->yMoy < depthCropTop || object->yMoy >= (depthHeight -depthCropBottom)) { delete object; }
					else { newObjectList->push_back(object); }
				}
				else { delete object; }
			}
			i++;
		}
	}
	delete foundFrame;
}


void Kinect::displaySensor (cv::Mat *depthFrame)
{
	if (realPositioning) {
		for (ObjectList::iterator it = newObjectList->begin(); it != newObjectList->end(); ++it)
		{
			Object *object = *it;
		
			std::stringstream ss;
			ss << object->index;
			std::string strIndex = ss.str();
			cv::putText(*depthFrame, strIndex, cv::Point(object->xMoy,object->yMoy), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(4500), 3);

			ss.str("");
			ss << round((object->rxMoy - object->rxMin)*100)/100;
			strIndex = ss.str();
			cv::putText(*depthFrame, strIndex, cv::Point(object->xMin,object->yMoy), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(4500), 2);

			ss.str("");
			ss << round((object->rxMax - object->rxMoy)*100)/100;
			strIndex = ss.str();
			cv::putText(*depthFrame, strIndex, cv::Point(object->xMax,object->yMoy), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(4500), 2);

			ss.str("");
			ss << round((object->ryMoy - object->ryMin)*100)/100;
			strIndex = ss.str();
			cv::putText(*depthFrame, strIndex, cv::Point(object->xMoy,object->yMin), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(4500), 2);

			ss.str("");
			ss << round((object->ryMax - object->ryMoy)*100)/100;
			strIndex = ss.str();
			cv::putText(*depthFrame, strIndex, cv::Point(object->xMoy,object->yMax), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(4500), 2);

			ss.str("");
			ss << round((object->rzMoy - object->rzMin)*100)/100;
			strIndex = ss.str();
			cv::putText(*depthFrame, strIndex, cv::Point(10,object->zMin), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(4500), 2);

			ss.str("");
			ss << round((object->rzMax - object->rzMoy)*100)/100;
			strIndex = ss.str();
			cv::putText(*depthFrame, strIndex, cv::Point(10,object->zMax), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(4500), 2);

			ss.str("");
			ss << "(" << round(object->rxMoy*100)/100 << ", " << round(object->ryMoy*100)/100 << ", " << round(object->rzMoy*100)/100 << ") -> " << round(object->ratio*100)/100;
			strIndex = ss.str();
			cv::putText(*depthFrame, strIndex, cv::Point(10,20), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(4500), 2);

			// ss.str("");
			// ss << "(" << round(object->xMoyS) << ", " << round(object->yMoyS) << ", " << round(object->zMoyS) << ")";
			// strIndex = ss.str();
			// cv::putText(*depthFrame, strIndex, cv::Point(10,50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(4500), 2);
		}
	}

	else {
		for (ObjectList::iterator it = newObjectList->begin(); it != newObjectList->end(); ++it)
		{
			Object *object = *it;
		
			std::stringstream ss;
			ss << object->index;
			std::string strIndex = ss.str();
			cv::putText(*depthFrame, strIndex, cv::Point(object->xMoy,object->yMoy), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(4500), 3);

			ss.str("");
			ss << round(object->xMoy - object->xMin);
			strIndex = ss.str();
			cv::putText(*depthFrame, strIndex, cv::Point(object->xMin,object->yMoy), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(4500), 2);

			ss.str("");
			ss << round(object->xMax - object->xMoy);
			strIndex = ss.str();
			cv::putText(*depthFrame, strIndex, cv::Point(object->xMax,object->yMoy), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(4500), 2);

			ss.str("");
			ss << round(object->yMoy - object->yMin);
			strIndex = ss.str();
			cv::putText(*depthFrame, strIndex, cv::Point(object->xMoy,object->yMin), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(4500), 2);

			ss.str("");
			ss << round(object->yMax - object->yMoy);
			strIndex = ss.str();
			cv::putText(*depthFrame, strIndex, cv::Point(object->xMoy,object->yMax), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(4500), 2);

			ss.str("");
			ss << round(object->zMoy - object->zMin);
			strIndex = ss.str();
			cv::putText(*depthFrame, strIndex, cv::Point(10,object->zMin), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(4500), 2);

			ss.str("");
			ss << round(object->zMax - object->zMoy);
			strIndex = ss.str();
			cv::putText(*depthFrame, strIndex, cv::Point(10,object->zMax), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(4500), 2);

			ss.str("");
			ss << "(" << round(object->xMoy) << ", " << round(object->yMoy) << ", " << round(object->zMoy) << ")";
			strIndex = ss.str();
			cv::putText(*depthFrame, strIndex, cv::Point(10,20), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(4500), 2);

			// ss.str("");
			// ss << "(" << round(object->xMoyS) << ", " << round(object->yMoyS) << ", " << round(object->zMoyS) << ")";
			// strIndex = ss.str();
			// cv::putText(*depthFrame, strIndex, cv::Point(10,50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(4500), 2);
		}
	}

	for (ObjectList::iterator it = newObjectList->begin(); it != newObjectList->end(); ++it)
	{
		Object *object = *it;
		
		for (int y = object->yMin; y <= object->yMax; y++)
		{
			depthFrame->at<float>(cv::Point(object->xMin,y)) = 4500;
			depthFrame->at<float>(cv::Point(object->xMoy,y)) = 4500;
			depthFrame->at<float>(cv::Point(object->xMax,y)) = 4500;
		}

		for (int x = object->xMin; x < object->xMax; x++)
		{
			depthFrame->at<float>(cv::Point(x,object->yMin)) = 4500;
			depthFrame->at<float>(cv::Point(x,object->yMoy)) = 4500;
			depthFrame->at<float>(cv::Point(x,object->yMax)) = 4500;
		}

		int zMiny = (object->zMin * depthHeight) / 400;
		int zMoyy = (object->zMoy * depthHeight) / 400;
		int zMaxy = (object->zMax * depthHeight) / 400;
		if (zMiny >= depthHeight) { zMiny--; }
		if (zMoyy >= depthHeight) { zMoyy--; }
		if (zMaxy >= depthHeight) { zMaxy--; }
	
		for (int x = 0; x <= 20; x++)
		{
			depthFrame->at<float>(cv::Point(x,zMiny)) = 4500;
			depthFrame->at<float>(cv::Point(x,zMoyy)) = 4500;
			depthFrame->at<float>(cv::Point(x,zMaxy)) = 4500;
		}
	}

	cv::resize (*depthFrame, *depthFrame, cv::Size(), 2, 2); 
	cv::imshow ("depth", *depthFrame / 4500.);
	//cv::resizeWindow ("depth", );
}


void Kinect::calibrateKinect (int key)
{
	bool newData = false;
	std::string axe = "";
	std::string type = "";

	if (key == 1048673) { axe = "x"; type = "Min"; newData = true; }
	if (key == 1048698) { axe = "x"; type = "Moy"; newData = true; }
	if (key == 1048677) { axe = "x"; type = "Max"; newData = true; }
	if (key == 1048689) { axe = "y"; type = "Min"; newData = true; }
	if (key == 1048691) { axe = "y"; type = "Moy"; newData = true; }
	if (key == 1048676) { axe = "y"; type = "Max"; newData = true; }
	if (key == 1048695) { axe = "z"; type = "Min"; newData = true; }
	if (key == 1048696) { axe = "z"; type = "Moy"; newData = true; }
	if (key == 1048675) { axe = "z"; type = "Max"; newData = true; }

	if (newData) {
		std::ofstream file;
		file.open("calibrate/calibrate.input", std::ios_base::app);

		for (ObjectList::iterator it = newObjectList->begin(); it != newObjectList->end(); ++it)
		{
			Object *object = *it;
				
			std::ostringstream stream;
			stream << object->xMoy << ";" << object->yMoy << ";" << object->zMoy << ";" << axe << ";" << type << ";";
			if (axe == "x") { stream << (object->xMoy - object->xMin) << ";" << (object->xMax - object->xMoy); }
			if (axe == "y") { stream << (object->yMoy - object->yMin) << ";" << (object->yMax - object->yMoy); }
			if (axe == "z") { stream << (object->zMoy - object->zMin) << ";" << (object->zMax - object->zMoy); }
			stream << std::endl;
			file << stream.str();
			std::cout << stream.str();
		}

		file.close();
	}

}


float Kinect::linearMap (float value, float min1, float max1, float min2, float max2) { return (value - min1) * (max2 - min2) / (max1 - min1) + min2; }


void Kinect::computeObjects ()
{
	for (ObjectList::iterator it = newObjectList->begin(); it != newObjectList->end(); ++it)
	{
		Object *object = *it;

		if (realPositioning) {
			if (fromAbove) {
				object->x = linearMap (object->rxMoy, xMin, xMax, graphicsWidth, 0);
				object->y = linearMap (object->ryMoy, yMin, yMax, graphicsHeight, 0);

				if (object->ratio < rMoy) { object->weight = linearMap (object->ratio, rMin, rMoy, weightMax, 0); }
				else { object->weight = linearMap (object->ratio, rMoy, rMax, 0, weightMin); }
			}
			
			else {
				object->x = linearMap (object->rxMoy, xMin, xMax, 0, graphicsWidth / sqrt (graphicsWidth * graphicsHeight));
				object->y = linearMap (object->rzMoy, zMin, zMax, graphicsHeight / sqrt (graphicsWidth * graphicsHeight), 0);

				if (object->ratio < rMoy) { object->weight = linearMap (object->ratio, rMin, rMoy, weightMax, 0); }
				else { object->weight = linearMap (object->ratio, rMoy, rMax, 0, weightMin); }
			}
				
			// std::cout << object->rxMoy << " " << object->rzMoy << " " << ratio << std::endl;
			// std::cout << object->x << " " << object->y << " " << object->weight << std::endl;
		}

		else {
			if (reverseXAxis) { object->x = linearMap(object->xMoy,0,depthHeight,graphicsWidth,0); }
			else { object->x = linearMap(object->xMoy,0,depthHeight,0,graphicsWidth); }

			if (reverseYAxis) { object->y = linearMap(object->zMoy,0,depthDepth,graphicsHeight,0); }
			else { object->y = linearMap(object->zMoy,0,depthDepth,0,graphicsHeight); }

			if (object->x < 0) { object->x = 0; } else if (object->x >= graphicsWidth) { object->x = graphicsWidth - 1; }
			if (object->y < 0) { object->y = 0; } else if (object->y >= graphicsHeight) { object->y = graphicsHeight - 1; }

			float xLeftMoy = 141.137681915898 + object->zMoy * -0.265896172366349 ;
			float xRightMoy = 153.394431112159 + object->zMoy * -0.290751630452789 ;
			float xRightMax = 229.83608352771 + object->zMoy * -0.444576937865905 ;
			float xLeftMax = 222.846660642123 + object->zMoy * -0.435883490904086 ;
			float xLeftMin = 88.1288073818904 + object->zMoy * -0.17052605260994 ;
			float xRightMin = 84.3813368345125 + object->zMoy * -0.157046334691957 ;
        
			float xLeft = object->xMoy - object->xMin;
			float xRight = object->xMax - object->xMoy;

			float xMin = xLeftMin + xRightMin;
			float xMoy = xLeftMoy + xRightMoy;
			float xMax = xLeftMax + xRightMax;

			float x = xLeft + xRight;
      
			if (x < xMoy) { object->weight = linearMap(x, xMin, xMoy, weightMax, 0); }
			else { object->weight = linearMap (x, xMoy, xMax, 0, weightMin); }
		}
	}
}



// int Kinect::ms_sleep (unsigned int ms)
// {
// 	int result = 0;

// 	{
// 		struct timespec ts_remaining =
// 			{ 
// 				ms / 1000, 
// 				(ms % 1000) * 1000000L 
// 			};

// 		do
// 		{
// 			struct timespec ts_sleep = ts_remaining;
// 			result = nanosleep(&ts_sleep, &ts_remaining);
// 		} 
// 		while (EINTR == result);
// 	}

// 	if (result)
// 	{
// 		perror("nanosleep() failed");
// 		result = -1;
// 	}

// 	return result;
// }

