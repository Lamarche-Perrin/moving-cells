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



// LIBRARIES

#include <iostream>
#include <signal.h>
#include <pthread.h>
#include <sys/time.h>

#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>


// PARAMETERS

const int delay = 80;
const bool initVertical = false;
const bool initReverse = true;

const int width = 1024;
const int height = 768;
const double switchingTime = 120;

const bool parallelComputation = false;
const int threadNumber = 6;


// VARIABLES

#define MILLION 1000000L;

int currentIndex;
int rowSize, colSize;
bool vertical, reverse;
bool stopKinect;

cv::Mat **matArray;
libfreenect2::Frame **rgbArray;
libfreenect2::FrameMap *framesArray;

cv::Mat *currentFrame;
uchar *currentPixel;

void *status;
pthread_attr_t attr;
pthread_t threads [threadNumber];
int firstDelay [threadNumber];
int lastDelay [threadNumber];

double timer;
struct timeval startTimer, endTimer;


// PRE-DEFINITION

void *computeVertical (void *arg);
void *computeVerticalReverse (void *arg);
void *computeHorizontal (void *arg);
void *computeHorizontalReverse (void *arg);

void serialVertical ();
void serialVerticalReverse ();
void serialHorizontal ();
void serialHorizontalReverse ();

	
// FUNCTIONS

void sigint_handler(int s) { stopKinect = true; }

int main(int argc, char *argv[])
{
	std::string program_path(argv[0]);
	size_t executable_name_idx = program_path.rfind("time-delays");

	std::string binpath = "/";

	if(executable_name_idx != std::string::npos) { binpath = program_path.substr(0, executable_name_idx); }

	libfreenect2::Freenect2 freenect2;

	libfreenect2::Freenect2Device *dev = 0;
	libfreenect2::PacketPipeline *pipeline = 0;

	if(freenect2.enumerateDevices() == 0) { std::cout << "no device connected!" << std::endl; return -1; }

	std::string serial = freenect2.getDefaultDeviceSerialNumber();

	pipeline = new libfreenect2::OpenGLPacketPipeline();
	dev = freenect2.openDevice(serial, pipeline);

	if(dev == 0) { std::cout << "failure opening device!" << std::endl; return -1; }

	signal(SIGINT,sigint_handler);
	stopKinect = false;

	//libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
	libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color);

	dev->setColorFrameListener(&listener);
    //dev->setIrAndDepthFrameListener(&listener);
	dev->start();

	std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
	std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
	cv::namedWindow ("time-delays", cv::WINDOW_NORMAL);
	cv::setWindowProperty ("time-delays", CV_WND_PROP_FULLSCREEN, 1);

	int frameNb = 0;
	vertical = initVertical;
	reverse = initReverse;
	stopKinect = false;
	
	matArray = new cv::Mat* [delay];
	rgbArray = new libfreenect2::Frame* [delay];
	framesArray = new libfreenect2::FrameMap [delay];

	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

	int delayPerThread = delay / threadNumber;
	int currentDelay = 0;
	for (int i = 0; i < threadNumber; i++)
	{
		firstDelay[i] = currentDelay;
		currentDelay += delayPerThread;
		lastDelay[i] = currentDelay;
	}
	firstDelay[0] = 1;
	lastDelay[threadNumber-1] = delay;

	rowSize = ((float)height/(float)delay);
	colSize = ((float)width/(float)delay);
		
	for (int d = 0; d < delay; d++)
	{
		listener.release(framesArray[d]);
		listener.waitForNewFrame(framesArray[d]);
		rgbArray[d] = (framesArray[d])[libfreenect2::Frame::Color];
		matArray[d] = new cv::Mat (rgbArray[d]->height, rgbArray[d]->width, CV_8UC4, rgbArray[d]->data);
		cv::resize (*matArray[d], *matArray[d], cv::Size(width,height));

		std::cout << "INIT: " << frameNb << " (" << d << ")" << std::endl;
		frameNb++;
	}

	int kinectFrameCounter = 0;
	double kinectSumDelay = 0;

	gettimeofday(&startTimer, NULL);

	while (!stopKinect)
	{
		gettimeofday(&endTimer, NULL);
		double t = (endTimer.tv_sec - startTimer.tv_sec) + (float)(endTimer.tv_usec - startTimer.tv_usec) / MILLION;
		startTimer = endTimer;
		timer += t;

		kinectFrameCounter++;
		kinectSumDelay += t;

		if (kinectSumDelay >= 3)
		{
			std::cout << "KINECT: " << (int)(((float)kinectFrameCounter)/kinectSumDelay) << "fps" << std::endl;
			kinectSumDelay = 0;
			kinectFrameCounter = 0;
		}	

		currentIndex = frameNb % delay;
		delete matArray[currentIndex];
		listener.release(framesArray[currentIndex]);

		listener.waitForNewFrame(framesArray[currentIndex]);
		rgbArray[currentIndex] = (framesArray[currentIndex])[libfreenect2::Frame::Color];
		matArray[currentIndex] = new cv::Mat (rgbArray[currentIndex]->height, rgbArray[currentIndex]->width, CV_8UC4, rgbArray[currentIndex]->data);
		cv::resize (*matArray[currentIndex], *matArray[currentIndex], cv::Size(width,height));

		//std::cout << "FRAME: " << frameNb << " (" << currentIndex << ")" << std::endl;
		frameNb++;

		currentIndex++;
		if (currentIndex >= delay) { currentIndex = 0; }
		currentFrame = matArray[currentIndex];
		currentPixel = matArray[currentIndex]->ptr<uchar>(0);

		if (parallelComputation)
		{
			for (int i = 0; i < threadNumber; i++)
			{
				int rc;
				if (vertical)
				{
					if (reverse) { rc = pthread_create(&threads[i], NULL, computeVerticalReverse, (void *) (intptr_t) i); }
					else { rc = pthread_create(&threads[i], NULL, computeVertical, (void *) (intptr_t) i); }
				}
				else {
					if (reverse) { rc = pthread_create(&threads[i], NULL, computeHorizontalReverse, (void *) (intptr_t) i); }
					else { rc = pthread_create(&threads[i], NULL, computeHorizontal, (void *) (intptr_t) i); }
				}
				
				if (rc) { std::cout << "Error:unable to create thread," << rc << std::endl; exit(-1); }
			}

			for (int i = 0; i < threadNumber; i++)
			{
				int rc = pthread_join (threads[i], &status);
				if (rc) { std::cout << "Error:unable to join," << rc << std::endl; exit(-1); }
			}			
		}

		else {
			if (vertical)
			{
				if (reverse) { serialVerticalReverse(); }
				else { serialVertical(); }
			}
			else {
				if (reverse) { serialHorizontalReverse(); }
				else { serialHorizontal(); }
			}
		}


		//cv::GaussianBlur (*currentFrame, *currentFrame, cv::Size(7,7), 1.5, 1.5);
		//cv::flip(*currentFrame, *currentFrame, 1);
		cv::imshow ("time-delays", *currentFrame);
		
		int key = cv::waitKey(1);
		if (key > 0) { std::cout << "KINECT KEY PRESSED: " << key << std::endl; }
		stopKinect = stopKinect || (key > 0 && ((key & 0xFF) == 27));

		//std::cout << timer << std::endl;
		if (switchingTime > 0 && timer > switchingTime)
		{
			if (vertical) { reverse = !reverse; }
			vertical = !vertical;
			timer = 0;
		}
	}

	dev->stop();
	dev->close();

	return 0;
}


void *computeVertical (void *arg)
{
	int id = (intptr_t) arg;
	for (int d = firstDelay[id]; d < lastDelay[id]; d++)
	{
		int newIndex = currentIndex+d;
		if (newIndex >= delay) { newIndex = 0; }
		uchar *pixel = matArray[newIndex]->ptr<uchar>(0);
		float firstRow = d*rowSize;
		float lastRow = firstRow + rowSize;
			
		for (int r = (int)firstRow; r < (int)lastRow; r++)
		{
			int i = r*width*4;
			for (int c = 0; c < width; c++)
			{
				currentPixel[i] = pixel[i];
				currentPixel[i+1] = pixel[i+1];
				currentPixel[i+2] = pixel[i+2];
				currentPixel[i+3] = pixel[i+3];
				i += 4;
			}
		}
	}
	pthread_exit(NULL);
}


void *computeVerticalReverse (void *arg)
{
	int id = (intptr_t) arg;
	for (int d = firstDelay[id]; d < lastDelay[id]; d++)
	{		
		int newIndex = currentIndex+d;
		if (newIndex >= delay) { newIndex = 0; }
		uchar *pixel = matArray[newIndex]->ptr<uchar>(0);
		float firstRow = (delay-d)*rowSize;
		float lastRow = firstRow - rowSize;
			
		for (int r = (int)lastRow; r > (int)firstRow; r--)
		{
			int i = r*width*4;
			for (int c = 0; c < width; c++)
			{
				currentPixel[i] = pixel[i];
				currentPixel[i+1] = pixel[i+1];
				currentPixel[i+2] = pixel[i+2];
				currentPixel[i+3] = pixel[i+3];
			}
		}
	}
	pthread_exit(NULL);
}


void *computeHorizontal (void *arg)
{
	int id = (intptr_t) arg;
	int width4 = width*4;
	for (int d = firstDelay[id]; d < lastDelay[id]; d++)
	{		
		int newIndex = currentIndex+d;
		if (newIndex >= delay) { newIndex = 0; }
		uchar *pixel = matArray[newIndex]->ptr<uchar>(0);
		float firstCol = d*colSize;
		float lastCol = firstCol + colSize;
			
		for (int c = (int)firstCol; c < (int)lastCol; c++)
		{
			int i = c*4;
			for (int r = 0; r < height; r++)
			{
				currentPixel[i] = pixel[i];
				currentPixel[i+1] = pixel[i+1];
				currentPixel[i+2] = pixel[i+2];
				currentPixel[i+3] = pixel[i+3];
				i += width4;
			}
		}
	}
	pthread_exit(NULL);
}


void *computeHorizontalReverse (void *arg)
{
	int id = (intptr_t) arg;
	int width4 = width*4;
	for (int d = firstDelay[id]; d < lastDelay[id]; d++)
	{		
		int newIndex = currentIndex+d;
		if (newIndex >= delay) { newIndex = 0; }
		uchar *pixel = matArray[newIndex]->ptr<uchar>(0);
		float firstCol = (delay-d)*colSize;
		float lastCol = firstCol - colSize;
			
		for (int c = (int)lastCol; c > (int)firstCol; c--)
		{
			int i = c*4;
			for (int r = 0; r < height; r++)
			{
				currentPixel[i] = pixel[i];
				currentPixel[i+1] = pixel[i+1];
				currentPixel[i+2] = pixel[i+2];
				currentPixel[i+3] = pixel[i+3];
				i += width4;
			}
		}
	}
	pthread_exit(NULL);
}


void serialHorizontal ()
{
	for (int d = 1; d < delay; d++)
	{
		currentIndex++;
		if (currentIndex >= delay) { currentIndex = 0; }
		uchar *pixel = matArray[currentIndex]->ptr<uchar>(0);
		float firstRow = d*((float)height/(float)delay);
		float lastRow = firstRow + ((float)height/(float)delay);
			
		for (int r = (int)firstRow; r < (int)lastRow; r++)
		{
			int i = r*width*4;
			for (int c = 0; c < width; c++)
			{
				currentPixel[i] = pixel[i];
				currentPixel[i+1] = pixel[i+1];
				currentPixel[i+2] = pixel[i+2];
				currentPixel[i+3] = pixel[i+3];
				i += 4;
			}
		}
	}
}


void serialHorizontalReverse ()
{
	for (int d = 1; d < delay; d++)
	{
		currentIndex++;
		if (currentIndex >= delay) { currentIndex = 0; }
		uchar *pixel = matArray[currentIndex]->ptr<uchar>(0);
		float firstRow = (delay-d)*((float)height/(float)delay);
		float lastRow = firstRow - ((float)height/(float)delay);
			
		for (int r = (int)firstRow; r > (int)lastRow; r--)
		{
			int i = r*width*4;
			for (int c = 0; c < width; c++)
			{
				currentPixel[i] = pixel[i];
				currentPixel[i+1] = pixel[i+1];
				currentPixel[i+2] = pixel[i+2];
				currentPixel[i+3] = pixel[i+3];
				i += 4;
			}
		}
	}
}


void serialVertical ()
{
	int width4 = width*4;
	for (int d = 1; d < delay; d++)
	{
		currentIndex++;
		if (currentIndex >= delay) { currentIndex = 0; }
		uchar *pixel = matArray[currentIndex]->ptr<uchar>(0);
		float firstCol = d*colSize;
		float lastCol = firstCol + colSize;
			
		for (int c = (int)firstCol; c < (int)lastCol; c++)
		{
			int i = c*4;
			for (int r = 0; r < height; r++)
			{
				currentPixel[i] = pixel[i];
				currentPixel[i+1] = pixel[i+1];
				currentPixel[i+2] = pixel[i+2];
				currentPixel[i+3] = pixel[i+3];
				i += width4;
			}
		}
	}
}


void serialVerticalReverse ()
{
	int width4 = width*4;
	for (int d = 1; d < delay; d++)
	{		
		currentIndex++;
		if (currentIndex >= delay) { currentIndex = 0; }
		uchar *pixel = matArray[currentIndex]->ptr<uchar>(0);
		float firstCol = (delay-d)*colSize;
		float lastCol = firstCol - colSize;
			
		for (int c = (int)firstCol; c > (int)lastCol; c--)
		{
			int i = c*4;
			for (int r = 0; r < height; r++)
			{
				currentPixel[i] = pixel[i];
				currentPixel[i+1] = pixel[i+1];
				currentPixel[i+2] = pixel[i+2];
				currentPixel[i+3] = pixel[i+3];
				i += width4;
			}
		}
	}
}
