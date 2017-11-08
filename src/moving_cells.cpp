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
#include <cstdio>
#include <signal.h>
#include <fstream>
#include <sys/time.h>
#include <errno.h>
#include <pthread.h>

#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>

#include "moving_cells.hpp"


// KINECT PARAMETERS

const float thresholdAdd = 100;
const int bodyMinSize = 1000;
const bool reverseXAxis = false;
const bool reverseYAxis = true;

const bool thresholdFromFile = true;

const bool allowKinectCalibration = false;
const bool allowSensorDisplay = false;
const bool allowGraphicsDisplay = true;



// GRAPHICS PARAMETERS

const int gravitationType = SYMMETRIC_GRAVITATION;
const int borderType = MIRROR_BORDER;
const int initType = UNIFORM_INIT;
const bool useSpeed = false;
const float xSpeedThreshold = 400.;

const float initialBodyAttractPower = 2;
const float initialBodyRepelPower = 4;
const float initialGravitationFactor = 1;
const float initialGravitationRadius = 0;
const float initialGravitationSpeed = 0.4;

const int initialParticleNumber = 1024*768/2;
const float initialParticleDamping = 0.02;

const int initialColor = 0;
const float initialParticleIntensity = 3;

const int initialThreadNumber = 6;

const int colorNb = 4;
const int particleRedArray[]   = {3, 12,  6, 7};
const int particleGreenArray[] = {6,  6,  5, 7};
const int particleBlueArray[]  = {12, 3, 12, 7};



/*
const bool useSpeed = true;
const float xSpeedThreshold = 400.;

const int gravitationType = EXPONENTIAL_GRAVITATION;
const float bodyAttractPower = 2.;
const float bodyRepelPower = 3.;
const float gravitationFactor = 1;
const float gravitationRadius = 0;
const float gravitationSpeed = 0.3;

const int particleNumber = 800000;
const float particleDamping = 0.02;

const float particleIntensity = 2;
const int particleRed = 5;
const int particleGreen = 3;
const int particleBlue = 7;

const int borderType = MIRROR_BORDER;
const int initType = UNIFORM_INIT;
*/

/*
// INTERACTIONS
const bool useSpeed = false;
const float xSpeedThreshold = 400.;

const int gravitationType = SYMMETRIC_GRAVITATION;
const float bodyAttractPower = 2.;
const float bodyRepelPower = 3.;
const float gravitationFactor = 1;
const float gravitationRadius = 300;
const float gravitationSpeed = 0.5;

const int particleNumber = 500000;
const float particleDamping = 0.02;

const float particleIntensity = 4;
const int particleRed = 3;
const int particleGreen = 7;
const int particleBlue = 13;

const int borderType = MIRROR_BORDER;
const int initType = UNIFORM_INIT;
*/


// KINECT VARIABLES

const int graphicsWidth = 1024;
const int graphicsHeight = 768;

const int depthWidth = 512;
const int depthHeight = 424;
const int depthDepth = 400;

int color = initialColor;
int particleRed = particleRedArray[color];
int particleGreen = particleGreenArray[color];
int particleBlue = particleBlueArray[color];

int particleNumber = initialParticleNumber;
int threadNumber = initialThreadNumber;

float gravitationFactor = initialGravitationFactor;
float gravitationRadius = initialGravitationRadius;
float gravitationSpeed = initialGravitationSpeed;
float particleIntensity = initialParticleIntensity;
float particleDamping = initialParticleDamping;
float bodyAttractPower = initialBodyAttractPower;
float bodyRepelPower = initialBodyRepelPower;

int bodyCounter;
bool stopKinect;
bool thresholdKinect;

cv::Mat *thresholdFrame;
BodyList *bodyList;
BodyList *newBodyList;
BodyList *currentBodyList;

bool verbose = false;
int kinectFps = 0;
int graphicsFps = 0;

// GRAPHICS VARIABLES

#define MILLION 1000000L;

const int pixelNumber = graphicsWidth*graphicsHeight;
struct timeval startTimer, endTimer;
int frameCounter;
double delay;
double sumDelay = 0;

Particle *particles;
int *pixels;
cv::Mat *frame;

int *redColor;
int *greenColor;
int *blueColor;

bool stop;
float bodyX, bodyY, bodyWeight;
float bodyLeftWeight, bodyRightWeight, bodyTopWeight, bodyBottomWeight;


// THREAD VARIABLES

void *status;
pthread_attr_t attr;
pthread_mutex_t mutex;

pthread_t loopThread;
pthread_t threads [initialThreadNumber];
int firstParticle [initialThreadNumber];
int lastParticle [initialThreadNumber];
int firstPixel [initialThreadNumber];
int lastPixel [initialThreadNumber];


// FUNCTIONS

int main (int argc, char *argv[])
{
	std::string program_path (argv[0]);
	size_t executable_name_idx = program_path.rfind ("moving-cells");

	std::string binpath = "/";

	if (executable_name_idx != std::string::npos)
	{
		binpath = program_path.substr (0, executable_name_idx);
	}

	libfreenect2::Freenect2 freenect2;

	if (freenect2.enumerateDevices() == 0)
	{
		std::cout << "no device connected!" << std::endl;
		return -1;
	}

	std::string serial = freenect2.getDefaultDeviceSerialNumber();
	libfreenect2::PacketPipeline *pipeline = new libfreenect2::OpenGLPacketPipeline();
	libfreenect2::Freenect2Device *dev = freenect2.openDevice (serial, pipeline);

	bool viewer_enabled = true;

	if (dev == 0)
	{
		std::cout << "failure opening device!" << std::endl;
		return -1;
	}

	signal (SIGINT,sigint_handler);
	stopKinect = false;

	libfreenect2::SyncMultiFrameListener listener (libfreenect2::Frame::Depth);
	libfreenect2::FrameMap frames;

	dev->setIrAndDepthFrameListener (&listener);
	dev->start();

	std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
	std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

	size_t framecount = 0;

	
	if (allowKinectCalibration)
	{
		std::ofstream file;
		file.open ("calibrate/calibrate.input", std::ios::out | std::ios::trunc);
		file << "X;Y;Z;AXE;TYPE;V1;V2" << std::endl;
		file.close();
	}

	bodyCounter = 0;
	
    thresholdFrame = 0;
    bodyList = new BodyList();
	currentBodyList = bodyList;
    newBodyList = 0;

	struct timeval kinectStartTimer, kinectEndTimer;
	gettimeofday(&kinectStartTimer, NULL);

	int kinectFrameCounter = 0;
	double kinectDelay = 0;
	double kinectSumDelay = 0;

	stopKinect = false;

	if (thresholdFromFile)
	{
		thresholdFrame = new cv::Mat();
		cv::FileStorage file ("threshold.ext", cv::FileStorage::READ);
		file["threshold"] >> (*thresholdFrame);
		file.release();
		thresholdKinect = false;
	}
	else { thresholdKinect = true; }

	if (allowGraphicsDisplay)
	{
		int rc = pthread_create(&loopThread, NULL, loop, NULL);
		if (rc) { std::cout << "Error:unable to create thread," << rc << std::endl; exit(-1); }
	}

	while (!stopKinect)
	{
		// COMPUTE TIME
		gettimeofday(&kinectEndTimer, NULL);
		kinectDelay = (kinectEndTimer.tv_sec - kinectStartTimer.tv_sec) + (float)(kinectEndTimer.tv_usec - kinectStartTimer.tv_usec) / MILLION;
		kinectStartTimer = kinectEndTimer;

		kinectFrameCounter++;
		kinectSumDelay += kinectDelay;

		if (kinectSumDelay >= 3)
		{
			kinectFps = (int)(((float)kinectFrameCounter)/kinectSumDelay);
			std::cout << "KINECT: " << kinectFps << "fps" << std::endl;
			kinectSumDelay = 0;
			kinectFrameCounter = 0;
		}	

		// GET NEW FRAME
		listener.waitForNewFrame(frames);
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
		cv::Mat *depthFrame = new cv::Mat (depth->height, depth->width, CV_32FC1, depth->data);

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
			if (tPixel[i] != 0 && dPixel[i] > tPixel[i]) { dPixel[i] = 0; }


		// DETECT BODIES
		extractBodies(dPixel);

		for (BodyList::iterator it = bodyList->begin(); it != bodyList->end(); ++it)
		{
			Body *body = *it;
			body->getClosestBody(newBodyList);
		}

		for (BodyList::iterator it = newBodyList->begin(); it != newBodyList->end(); ++it)
		{
			Body *body = *it;
			body->update(kinectDelay);
		}

		pthread_mutex_lock(&mutex);
		if (currentBodyList != bodyList)
		{
			for (BodyList::iterator it = bodyList->begin(); it != bodyList->end(); ++it) { delete (*it); }
			delete bodyList;
		}
		bodyList = newBodyList;
		pthread_mutex_unlock(&mutex);

		
		// DISPLAY SENSOR
		if (allowSensorDisplay) { displaySensor(depthFrame); }

		int key = cv::waitKey(1);
		if (key > 0)
		{
			key = key & 0xFF;
			std::cout << "KINECT KEY PRESSED: " << key << std::endl;

			stopKinect = stopKinect || key == 27; // ESC escape
			thresholdKinect = thresholdKinect || key == 8; // BS backspace

			if (allowKinectCalibration) { calibrateKinect(key); }
		}

		listener.release(frames);
		delete depthFrame;
		//libfreenect2::this_thread::sleep_for(libfreenect2::chrono::milliseconds(100));
	}

	dev->stop();
	dev->close();

	return 0;
}


void sigint_handler (int s) { stopKinect = true; }

int scale (float z) { return round((z-500)/10); }


Body::Body () {
	closestBody = 0;
	minDist = -1;
}


void Body::getClosestBody (BodyList *list) {
	this->closestBody = 0;
	this->minDist = -1;
	for (BodyList::iterator it = list->begin(); it != list->end(); ++it)
	{
		Body *newBody = *it;
		double dist = this->getDistance(newBody);
		if (newBody->minDist < 0 || newBody->minDist > dist)
		{
			if (this->closestBody == 0 || dist < minDist)
			{
				this->closestBody = newBody;
				this->minDist = dist;
			}
		}
	}
	
	if (this->closestBody != 0)
	{
		Body *oldBody = this->closestBody->closestBody;
		this->closestBody->closestBody = this;
		this->closestBody->minDist = this->minDist;
		if (oldBody != 0) { oldBody->getClosestBody(list); }
	}
}


void Body::update (double delay) {
	if (this->closestBody == 0)
	{
		this->index = bodyCounter++;
		xMinS = 0; xMoyS = 0; xMaxS = 0; yMinS = 0; yMoyS = 0; yMaxS = 0; zMinS = 0; zMoyS = 0; zMaxS = 0;
	}

	else {
		Body *b = this->closestBody;
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
	this->closestBody = 0;
	this->minDist = -1;
}


double Body::getDistance (Body *body) {
	return sqrt( pow(this->xMoy-body->xMoy,2) + pow(this->yMoy-body->yMoy,2) + pow(this->zMoy-body->zMoy,2));
}


void Body::print ()
{
	std::cout << "(" << xMin << "," << xMoy << "," << xMax << ") "
			  << "(" << yMin << "," << yMoy << "," << yMax << ") "
			  << "(" << zMin << "," << zMoy << "," << zMax << ")" << std::endl;
}


void extractBodies (float *dPixel)
{
	newBodyList = new BodyList();
	
	cv::Mat *foundFrame = new cv::Mat (depthHeight, depthWidth, CV_32FC1, double(0));
	float *fPixel = foundFrame->ptr<float>(0);
	for (int i = 0; i < depthWidth*depthHeight; i++)
	{
		if (dPixel[i] > 0 && fPixel[i] == 0)
		{
			int x = i % depthWidth;
			int y = (i - x) / depthWidth;
			int z = scale(dPixel[i]);	    

			Body *body = new Body();
			body->pixelNb = 0;
			body->xMin = x;
			body->xMoy = 0;
			body->xMax = x;
			body->yMin = y;
			body->yMoy = 0;
			body->yMax = y;
			body->zMin = z;
			body->zMoy = 0;
			body->zMax = z;
			body->closestBody = 0;
			body->minDist = -1;

			std::list<Pixel> pixelList;
			pixelList.push_back(Pixel(x,y,z));
			foundFrame->at<float>(cv::Point(x,y)) = 1;
		    
			while (!pixelList.empty())
			{
				Pixel p = pixelList.front();
				pixelList.pop_front();

				body->pixelNb++;
				body->xMoy += p.x;
				body->yMoy += p.y;
				body->zMoy += p.z;

				if (p.x < body->xMin) { body->xMin = p.x; }
				if (p.x > body->xMax) { body->xMax = p.x; }
				if (p.y < body->yMin) { body->yMin = p.y; }
				if (p.y > body->yMax) { body->yMax = p.y; }
				if (p.z < body->zMin) { body->zMin = p.z; }
				if (p.z > body->zMax) { body->zMax = p.z; }

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

			body->xMoy /= body->pixelNb;
			body->yMoy /= body->pixelNb;
			body->zMoy /= body->pixelNb;
			if (body->pixelNb >= bodyMinSize) { newBodyList->push_back(body); }
			else { delete body; }
		}
	}
	delete foundFrame;
}


void displaySensor (cv::Mat *depthFrame)
{
	for (BodyList::iterator it = bodyList->begin(); it != bodyList->end(); ++it)
	{
		Body *body = *it;
		
		std::stringstream ss;
		ss << body->index;
		std::string strIndex = ss.str();
		cv::putText(*depthFrame, strIndex, cv::Point(body->xMoy,body->yMoy), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(4500), 3);

		ss.str("");
		ss << round(body->xMoy - body->xMin);
		strIndex = ss.str();
		cv::putText(*depthFrame, strIndex, cv::Point(body->xMin,body->yMoy), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(4500), 2);

		ss.str("");
		ss << round(body->xMax - body->xMoy);
		strIndex = ss.str();
		cv::putText(*depthFrame, strIndex, cv::Point(body->xMax,body->yMoy), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(4500), 2);

		ss.str("");
		ss << round(body->yMoy - body->yMin);
		strIndex = ss.str();
		cv::putText(*depthFrame, strIndex, cv::Point(body->xMoy,body->yMin), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(4500), 2);

		ss.str("");
		ss << round(body->yMax - body->yMoy);
		strIndex = ss.str();
		cv::putText(*depthFrame, strIndex, cv::Point(body->xMoy,body->yMax), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(4500), 2);

		ss.str("");
		ss << round(body->zMoy - body->zMin);
		strIndex = ss.str();
		cv::putText(*depthFrame, strIndex, cv::Point(10,body->zMin), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(4500), 2);

		ss.str("");
		ss << round(body->zMax - body->zMoy);
		strIndex = ss.str();
		cv::putText(*depthFrame, strIndex, cv::Point(10,body->zMax), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(4500), 2);

		ss.str("");
		ss << "(" << round(body->xMoy) << ", " << round(body->yMoy) << ", " << round(body->zMoy) << ")";
		strIndex = ss.str();
		cv::putText(*depthFrame, strIndex, cv::Point(10,20), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(4500), 2);

		ss.str("");
		ss << "(" << round(body->xMoyS) << ", " << round(body->yMoyS) << ", " << round(body->zMoyS) << ")";
		strIndex = ss.str();
		cv::putText(*depthFrame, strIndex, cv::Point(10,50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(4500), 2);


		for (int y = body->yMin; y <= body->yMax; y++)
		{
			depthFrame->at<float>(cv::Point(body->xMin,y)) = 4500;
			depthFrame->at<float>(cv::Point(body->xMoy,y)) = 4500;
			depthFrame->at<float>(cv::Point(body->xMax,y)) = 4500;
		}

		for (int x = body->xMin; x < body->xMax; x++)
		{
			depthFrame->at<float>(cv::Point(x,body->yMin)) = 4500;
			depthFrame->at<float>(cv::Point(x,body->yMoy)) = 4500;
			depthFrame->at<float>(cv::Point(x,body->yMax)) = 4500;
		}

		int zMiny = (body->zMin * depthHeight) / 400;
		int zMoyy = (body->zMoy * depthHeight) / 400;
		int zMaxy = (body->zMax * depthHeight) / 400;
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
	    
	cv::imshow ("depth", *depthFrame / 4500.);
}


void calibrateKinect (int key)
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

		for (BodyList::iterator it = bodyList->begin(); it != bodyList->end(); ++it)
		{
			Body *body = *it;
				
			std::ostringstream stream;
			stream << body->xMoy << ";" << body->yMoy << ";" << body->zMoy << ";" << axe << ";" << type << ";";
			if (axe == "x") { stream << (body->xMoy - body->xMin) << ";" << (body->xMax - body->xMoy); }
			if (axe == "y") { stream << (body->yMoy - body->yMin) << ";" << (body->yMax - body->yMoy); }
			if (axe == "z") { stream << (body->zMoy - body->zMin) << ";" << (body->zMax - body->zMoy); }
			stream << std::endl;
			file << stream.str();
			std::cout << stream.str();
		}

		file.close();
	}

}


void *loop (void *arg)
{
	setup();
	while (!stop)
	{
		getTime();
		computeBodies();
		computeParticles();
		draw();
	}
}


void setup ()
{
	srand(time(0));
	stop = false;

	// SETUP PARTICLES
	particles = new Particle [initialParticleNumber];
	initParticles(initType);

	redColor = new int [initialParticleNumber+1];
	blueColor = new int [initialParticleNumber+1];
	greenColor = new int [initialParticleNumber+1];

	setupColor(true);
	
	// SETUP DISPLAY
	cv::namedWindow("moving-cells", CV_WINDOW_NORMAL);
	cv::setWindowProperty ("moving-cells", CV_WND_PROP_FULLSCREEN, 1);
	
	frame = new cv::Mat(graphicsHeight, graphicsWidth, CV_8UC3);
	pixels = new int [graphicsWidth*graphicsHeight];

	// SETUP THREADS
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

	setupThreads();

	// SETUP TIME
	frameCounter = 0;
	delay = 0;
	sumDelay = 0;
	gettimeofday(&startTimer, NULL);
}


void setupThreads ()
{
	int particlePerThread = particleNumber / threadNumber;
	int currentParticle = 0;
	for (int i = 0; i < threadNumber; i++)
	{
		firstParticle[i] = currentParticle;
		currentParticle += particlePerThread;
		lastParticle[i] = currentParticle;
	}
	lastParticle[threadNumber-1] = particleNumber;

	int pixelPerThread = pixelNumber / threadNumber;
	int currentPixel = 0;
	for (int i = 0; i < threadNumber; i++)
	{
		firstPixel[i] = currentPixel;
		currentPixel += pixelPerThread;
		lastPixel[i] = currentPixel;
	}
	lastPixel[threadNumber-1] = pixelNumber;
}


void setupColor (bool init)
{
	if (init)
	{
		particleRed = particleRedArray[color];
		particleGreen = particleGreenArray[color];
		particleBlue = particleBlueArray[color];
	}

	for (int i = 0; i < particleNumber; i++)
	{
		redColor[i] = std::min((int)(i*particleRed*particleIntensity),255);
		blueColor[i] = std::min((int)(i*particleBlue*particleIntensity),255);
		greenColor[i] = std::min((int)(i*particleGreen*particleIntensity),255);
	}
}


void getTime()
{
	gettimeofday(&endTimer, NULL);
	delay = (endTimer.tv_sec - startTimer.tv_sec) + (float)(endTimer.tv_usec - startTimer.tv_usec) / MILLION;
	startTimer = endTimer;

	frameCounter++;
	sumDelay += delay;

	if (sumDelay >= 3)
	{
		graphicsFps = (int)(((float)frameCounter)/sumDelay);
		std::cout << "GRAPHICS: " << graphicsFps << "fps" << std::endl;
		sumDelay = 0;
		frameCounter = 0;
	}	
}


void initParticles (int type)
{
	switch (type)
	{
	case RANDOM_INIT :
	{
		for (int i = 0; i < initialParticleNumber; i++)
		{
			Particle *particle = &particles[i];
			particle->px = rand() % graphicsWidth;
			particle->py = rand() % graphicsHeight;
			particle->dx = particle->dy = 0;
		}	
	}
	break;

	case UNIFORM_INIT :
	{
		int index = 0;
		float bin = sqrt((float)(graphicsWidth*graphicsHeight)/particleNumber);
		for (float x = bin/2; x <= graphicsWidth - bin/2; x += bin)
		{
			for (float y = bin/2; y <= graphicsHeight - bin/2; y += bin)
			{
				Particle *particle = &particles[index++];
				particle->px = x;
				particle->py = y;
				particle->dx = particle->dy = 0;
			}
		}

		for (int i = index; i < initialParticleNumber; i++) {
			Particle *particle = &particles[i];
			particle->px = rand() % graphicsWidth;
			particle->py = rand() % graphicsHeight;
			particle->dx = particle->dy = 0;
		}
	}
	break;
	}	
}


float linearMap (float value, float min1, float max1, float min2, float max2) { return (value - min1) * (max2 - min2) / (max1 - min1) + min2; }


void computeBodies ()
{
	pthread_mutex_lock(&mutex);
	if (currentBodyList != bodyList)
	{
		for (BodyList::iterator it = currentBodyList->begin(); it != currentBodyList->end(); ++it) { delete (*it); }
		delete currentBodyList;
	}
	currentBodyList = bodyList;
	pthread_mutex_unlock(&mutex);

	for (BodyList::iterator it = currentBodyList->begin(); it != currentBodyList->end(); ++it)
	{
		Body *body = *it;

		if (reverseXAxis) { bodyX = linearMap(body->xMoy,0,depthHeight,graphicsWidth,0); }
		else { bodyX = linearMap(body->xMoy,0,depthHeight,0,graphicsWidth); }

		if (reverseYAxis) { bodyY = linearMap(body->zMoy,0,depthDepth,graphicsHeight,0); }
		else { bodyY = linearMap(body->zMoy,0,depthDepth,0,graphicsHeight); }

		if (bodyX < 0) { bodyX = 0; } else if (bodyX >= graphicsWidth) { bodyX = graphicsWidth - 1; }
		if (bodyY < 0) { bodyY = 0; } else if (bodyY >= graphicsHeight) { bodyY = graphicsHeight - 1; }

		switch (gravitationType) {
        
		case SYMMETRIC_GRAVITATION :
		{
			if (useSpeed) {
				float speed = (body->xMaxS-body->xMinS)/2;
				if (speed > xSpeedThreshold) { bodyWeight = linearMap(speed,0,200,0,-bodyRepelPower); }
				else {
					float xLeftMoy = 141.137681915898 + body->zMoy * -0.265896172366349 ;
					float xRightMoy = 153.394431112159 + body->zMoy * -0.290751630452789 ;
					float xRightMax = 229.83608352771 + body->zMoy * -0.444576937865905 ;
					float xLeftMax = 222.846660642123 + body->zMoy * -0.435883490904086 ;
					float xLeftMin = 88.1288073818904 + body->zMoy * -0.17052605260994 ;
					float xRightMin = 84.3813368345125 + body->zMoy * -0.157046334691957 ;

					float xLeft = body->xMoy - body->xMin;
					float xRight = body->xMax - body->xMoy;

					float xMin = xLeftMin + xRightMin;
					float xMoy = xLeftMoy + xRightMoy;
					float xMax = xLeftMax + xRightMax;

					float x = xLeft + xRight;
      
					if (x < xMoy) { bodyWeight = linearMap(x,xMin,xMoy,bodyAttractPower,0); }
				}
			}

			else {
				float xLeftMoy = 141.137681915898 + body->zMoy * -0.265896172366349 ;
				float xRightMoy = 153.394431112159 + body->zMoy * -0.290751630452789 ;
				float xRightMax = 229.83608352771 + body->zMoy * -0.444576937865905 ;
				float xLeftMax = 222.846660642123 + body->zMoy * -0.435883490904086 ;
				float xLeftMin = 88.1288073818904 + body->zMoy * -0.17052605260994 ;
				float xRightMin = 84.3813368345125 + body->zMoy * -0.157046334691957 ;
        
				float xLeft = body->xMoy - body->xMin;
				float xRight = body->xMax - body->xMoy;

				float xMin = xLeftMin + xRightMin;
				float xMoy = xLeftMoy + xRightMoy;
				float xMax = xLeftMax + xRightMax;

				float x = xLeft + xRight;
      
				if (x < xMoy) { bodyWeight = linearMap(x,xMin,xMoy,bodyAttractPower,0); }
				else { bodyWeight = linearMap(x,xMoy,xMax,0,-bodyRepelPower); }
			}
		}
		break;

		case QUADRANT_GRAVITATION : case LINEAR_GRAVITATION : case EXPONENTIAL_GRAVITATION :
		{
			if (useSpeed) {
				float speed = (body->xMaxS-body->xMinS)/2;
				if (speed > xSpeedThreshold) { bodyLeftWeight = bodyRightWeight = bodyTopWeight = bodyBottomWeight = linearMap(speed,0,xSpeedThreshold,0,-2*bodyRepelPower); }
				else {
					float xLeftMoy = 141.137681915898 + body->zMoy * -0.265896172366349 ;
					float xRightMoy = 153.394431112159 + body->zMoy * -0.290751630452789 ;
					float xRightMax = 229.83608352771 + body->zMoy * -0.444576937865905 ;
					float xLeftMax = 222.846660642123 + body->zMoy * -0.435883490904086 ;
					float xLeftMin = 88.1288073818904 + body->zMoy * -0.17052605260994 ;
					float xRightMin = 84.3813368345125 + body->zMoy * -0.157046334691957 ;
					float yBottomMoy = 133.755606270363 + body->zMoy * -0.0983414952321892 + pow(body->zMoy,2) * -0.000271500935861165 ;
					float yBottomMax = 212.908568445793 + body->zMoy * -0.332923639449663 + pow(body->zMoy,2) * -8.81651485585909e-05 ;

					float yTopMoy = 109.337213570448 + body->zMoy * 0.114120900345369 + pow(body->zMoy,2) * -0.000684891973255737 ;
					float yTopMax = 191.951017587996 + body->zMoy * -0.232794494230743 + pow(body->zMoy,2) * -0.000246330076936845 ;
					float zFrontMax = 52.74426210174 + body->zMoy * -0.1929853192676 + pow(body->zMoy,2) * 0.00210624201544209 + pow(body->zMoy,3) * -4.1952134127603e-06 ;
					float zFrontMoy = 16.1450019204802 + body->zMoy * 0.233090101419243 + pow(body->zMoy,2) * -0.000188126196289432 + pow(body->zMoy,3) * -6.13212732132743e-07 ;
        
					float xLeft = body->xMoy - body->xMin;
					float xRight = body->xMax - body->xMoy;
					float yTop = body->yMax - body->yMoy;

					float xMin = xLeftMin + xRightMin;
					float xMoy = xLeftMoy + xRightMoy;
					float xMax = xLeftMax + xRightMax;

					float x = xLeft + xRight;
					float yThreshold = yTopMoy;
					//std::cout << yTopMoy << "   " << yTop << "   " << yTopMax << std::endl;

					if (yTop > yThreshold)
					{
						bodyLeftWeight = bodyRightWeight = 0;
						bodyTopWeight = bodyBottomWeight = linearMap(yTop,yThreshold,yTopMax,0,2*bodyAttractPower);
					}
					else if (x < xMoy) { bodyLeftWeight = bodyRightWeight = bodyTopWeight = bodyBottomWeight = linearMap(x,xMin,xMoy,bodyAttractPower,0); }

					/*
					//float yMin = yBottomMin + yTopMin;
					float yMoy = yBottomMoy + yTopMoy;
					float yMax = yBottomMax + yTopMax;

					float ratioMoy = xMoy/yMoy;
					float ratioMax = xMin/yMax;
					float ratioMin = yMax/xMin;
					float ratio = ((float)body->xMax - (float)body->xMin)/((float)body->yMax - (float)body->yMin);
					std::cout << ratioMin << "  " << ratio << "  " << ratioMax << std::endl;

					bodyTopWeight = bodyBottomWeight = linearMap(ratio,ratioMin,ratioMax,0,bodyAttractPower);
					bodyLeftWeight = bodyRightWeight = linearMap(ratio,ratioMin,ratioMax,bodyAttractPower,0);
					*/
				}
			}

			else {
				float xLeftMin = 91.5361166850005 + body->zMoy * -0.219373694097734 ;
				float xRightMin = 89.6285996303111 + body->zMoy * -0.213193855087035 ;
				float xLeftMoy = 182.523638698647 + body->zMoy * -0.432365782586038 ;
				float xRightMoy = 184.033174004946 + body->zMoy * -0.43261951092886 ;
				float xLeftMax = 286.673815920798 + body->zMoy * -0.653639390943621 ;
				float xRightMax = 226.730996551708 + body->zMoy * -0.463157907184652 ;
				float yTopMin = 152.596415677309 + body->zMoy * -0.322485767964055 ;
				float yBottomMin = 138.690675328497 + body->zMoy * -0.273156060074471 ;
				float yTopMoy = 215.663789080728 + body->zMoy * -0.392673821745217 ;
				float yBottomMoy = 232.626140770943 + body->zMoy * -0.402981877710885 ;
				float yTopMax = 230.171686137297 + body->zMoy * -0.345754294141461 ;
				float yBottomMax = 217.815525417847 + body->zMoy * -0.324352264183679 ;

				float xLeft = body->xMoy - body->xMin;
				float xRight = body->xMax - body->xMoy;

				float yTop = body->yMoy - body->yMin;
				float yBottom = body->yMax - body->yMoy;
				//float zFront = body->zMoy - body->zMin;
				//float zBack = body->zMax - body->zMoy;
      
				if (xLeft < xLeftMoy) { bodyLeftWeight = linearMap(xLeft,xLeftMin,xLeftMoy,bodyAttractPower,0); }
				else { bodyLeftWeight = linearMap(xLeft,xLeftMoy,xLeftMax,0,-bodyRepelPower); }

				if (xRight < xRightMoy) { bodyRightWeight = linearMap(xRight,xRightMin,xRightMoy,bodyAttractPower,0); }
				else { bodyRightWeight = linearMap(xRight,xRightMoy,xRightMax,0,-bodyRepelPower); }

				if (yTop < yTopMoy) { bodyTopWeight = linearMap(yTop,yTopMin,yTopMoy,bodyAttractPower,0); }
				else { bodyTopWeight = linearMap(yTop,yTopMoy,yTopMax,0,-bodyRepelPower); }

				if (yBottom < yBottomMoy) { bodyBottomWeight = linearMap(yBottom,yBottomMin,yBottomMoy,bodyAttractPower,0); }
				else { bodyBottomWeight = linearMap(yBottom,yBottomMoy,yBottomMax,0,-bodyRepelPower); }
			}
		}
		break;
		}

		// UPDATE PARTICLES
		for (int i = 0; i < threadNumber; i++)
		{
			int rc = pthread_create(&threads[i], NULL, updateParticles, (void *) (intptr_t) i);
			if (rc) { std::cout << "Error:unable to create thread," << rc << std::endl; exit(-1); }
		}

		for (int i = 0; i < threadNumber; i++)
		{
			int rc = pthread_join (threads[i], &status);
			if (rc) { std::cout << "Error:unable to join," << rc << std::endl; exit(-1); }
		}		
	}
}


void computeParticles ()
{
	// MOVE PARTICLES
	for (int i = 0; i < threadNumber; i++)
	{
		int rc = pthread_create(&threads[i], NULL, moveParticles, (void *) (intptr_t) i);
		if (rc) { std::cout << "Error:unable to create thread," << rc << std::endl; exit(-1); }
	}

	for (int i = 0; i < threadNumber; i++)
	{
		int rc = pthread_join (threads[i], &status);
		if (rc) { std::cout << "Error:unable to join," << rc << std::endl; exit(-1); }
	}

	// CLEAR PIXELS
	for (int i = 0; i < threadNumber; i++)
	{
		int rc = pthread_create(&threads[i], NULL, clearPixels, (void *) (intptr_t) i);
		if (rc) { std::cout << "Error:unable to create thread," << rc << std::endl; exit(-1); }
	}

	for (int i = 0; i < threadNumber; i++)
	{
		int rc = pthread_join (threads[i], &status);
		if (rc) { std::cout << "Error:unable to join," << rc << std::endl; exit(-1); }
	}
	
	// APPLY PARTICLES TO PIXELS
	for (int i = 0; i < threadNumber; i++)
	{
		int rc = pthread_create(&threads[i], NULL, applyParticles, (void *) (intptr_t) i);
		if (rc) { std::cout << "Error:unable to create thread," << rc << std::endl; exit(-1); }
	}

	for (int i = 0; i < threadNumber; i++)
	{
		int rc = pthread_join (threads[i], &status);
		if (rc) { std::cout << "Error:unable to join," << rc << std::endl; exit(-1); }
	}

	// CLEAR PIXELS TO FRAME
	for (int i = 0; i < threadNumber; i++)
	{
		int rc = pthread_create(&threads[i], NULL, applyPixels, (void *) (intptr_t) i);
		if (rc) { std::cout << "Error:unable to create thread," << rc << std::endl; exit(-1); }
	}

	for (int i = 0; i < threadNumber; i++)
	{
		int rc = pthread_join (threads[i], &status);
		if (rc) { std::cout << "Error:unable to join," << rc << std::endl; exit(-1); }
	}
}


void draw ()
{
	if (verbose)
	{
		int x = 10;
		int y = 20;
		
		std::stringstream ss;
		ss << "Radius: " << gravitationRadius;
		std::string str = ss.str();
		cv::putText(*frame, str, cv::Point(x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
		y += 20;

		ss.str("");
		ss << "Attract: " << bodyAttractPower;
		str = ss.str();
		cv::putText(*frame, str, cv::Point(x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
		y += 20;

		ss.str("");
		ss << "Repel: " << bodyRepelPower;
		str = ss.str();
		cv::putText(*frame, str, cv::Point(x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
		y += 20;

		ss.str("");
		ss << "Particles: " << particleNumber;
		str = ss.str();
		cv::putText(*frame, str, cv::Point(x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
		y += 20;

		ss.str("");
		ss << "Intensity: " << particleIntensity;
		str = ss.str();
		cv::putText(*frame, str, cv::Point(x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
		y += 20;

		ss.str("");
		ss << "Threads: " << threadNumber;
		str = ss.str();
		cv::putText(*frame, str, cv::Point(x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
		y += 20;

		ss.str("");
		ss << "Speed: " << gravitationSpeed;
		str = ss.str();
		cv::putText(*frame, str, cv::Point(x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
		y += 20;

		ss.str("");
		ss << "Damping: " << particleDamping;
		str = ss.str();
		cv::putText(*frame, str, cv::Point(x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
		y += 20;

		ss.str("");
		ss << "Factor: " << gravitationFactor;
		str = ss.str();
		cv::putText(*frame, str, cv::Point(x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
		y += 20;

		y += 10;
		ss.str("");
		ss << "Kinect: " << kinectFps << "fps";
		str = ss.str();
		cv::putText(*frame, str, cv::Point(x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
		y += 20;

		ss.str("");
		ss << "Graphics: " << graphicsFps << "fps";
		str = ss.str();
		cv::putText(*frame, str, cv::Point(x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
		y += 20;

		ss.str("");
		ss << "Color: " << particleRed << " " << particleGreen << " " << particleBlue;
		str = ss.str();
		cv::putText(*frame, str, cv::Point(x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
		y += 20;
	}
	
	//cv::GaussianBlur (*frame, *frame, cv::Size(1,1), 1.5, 1.5);
	cv::imshow("moving-cells", *frame);
	int key = cv::waitKey(1);
	if (key > 0)
	{
		key = key & 0xFF;
		std::cout << "KEY PRESSED: " << key << std::endl;
		stop = stop || key == 27; // ESC

		switch (key)
		{
		case 117 : // u
			initParticles(UNIFORM_INIT);
			break;
			
		case 106 : // j
			initParticles(RANDOM_INIT);
			break;

		case 141 : // Enter (vernum)
			verbose = !verbose;

			
			
		case 97 : // a
			gravitationRadius -= 100;
			if (gravitationRadius < 0) { gravitationRadius = 0; }
			break;

		case 122 : // z
			gravitationRadius = initialGravitationRadius;
			break;

		case 101 : // e
			gravitationRadius += 100;
			break;

			
			
		case 113 : // q
			bodyAttractPower -= 0.1;
			if (bodyAttractPower < 0) { bodyAttractPower = 0; }
			break;

		case 115 : // s
			bodyAttractPower = initialBodyAttractPower;
			break;

		case 100 : // d
			bodyAttractPower += 0.1;
			break;

			
			
		case 119 : // w
			bodyRepelPower -= 0.1;
			if (bodyRepelPower < 0) { bodyRepelPower = 0; }
			break;

		case 120 : // x
			bodyRepelPower = initialBodyRepelPower;
			break;

		case 99 : // c
			bodyRepelPower += 0.1;
			break;


			
		case 114 : // r
			particleNumber -= (int) ((float)graphicsWidth*graphicsHeight / 16);
			if (particleNumber < 0) { particleNumber += (int) ((float)graphicsWidth*graphicsHeight / 16); } 
			setupThreads();
			break;

		case 116 : // t
			particleNumber = initialParticleNumber;
			setupThreads();
			break;

		case 121 : // y
			particleNumber += (int) ((float)graphicsWidth*graphicsHeight / 16);
			if (particleNumber > initialParticleNumber) { particleNumber = initialParticleNumber; } 
			setupThreads();
			break;


			
		case 102 : // f
			particleIntensity -= 1;
			if (particleIntensity < 0) { particleIntensity = 0; } 
			setupColor(false);
			break;

		case 103 : // g
			particleIntensity = initialParticleIntensity;
			setupColor(false);
			break;

		case 104 : // h
			particleIntensity += 1;
			setupColor(false);
			break;

			

		case 118 : // v
			threadNumber -= 1;
			if (threadNumber < 1) { threadNumber = 1; } 
			setupThreads();
			break;

		case 98 : // b
			threadNumber = initialThreadNumber;
			setupThreads();
			break;

		case 110 : // n
			threadNumber += 1;
			if (threadNumber > initialThreadNumber) { threadNumber = initialThreadNumber; }
			setupThreads();
			break;


			
		case 183 : // 7 (vernum)
			gravitationSpeed -= 0.1;
			if (gravitationSpeed < 0) { gravitationSpeed = 0; } 
			break;

		case 184 : // 8 (vernum)
			gravitationSpeed = initialGravitationSpeed;
			break;

		case 185 : // 9 (vernum)
			gravitationSpeed += 0.1;
			break;
			

			
		case 180 : // 4 (vernum)
			particleDamping -= 0.002;
			if (particleDamping < 0) { particleDamping = 0; }
			break;

		case 181 : // 5 (vernum)
			particleDamping = initialParticleDamping;
			break;

		case 182 : // 6 (vernum)
			particleDamping += 0.002;
			break;

			
			
		case 177 : // 1 (vernum)
			gravitationFactor -= 0.1;
			break;

		case 178 : // 2 (vernum)
			gravitationFactor = initialGravitationFactor;
			break;

		case 179 : // 3 (vernum)
			gravitationFactor += 0.1;
			break;

			
		case 105 : // i
			color--;
			if (color < 0) { color = colorNb - 1; }
			setupColor(true);
			break;

		case 111 : // o
			color = initialColor;
			setupColor(true);
			break;

		case 112 : // p
			color++;
			if (color >= colorNb) { color = 0; }
			setupColor(true);
			break;

			
		case 107 : // k
			particleRed++;
			setupColor(false);
			break;

		case 108 : // l
			particleGreen++;
			setupColor(false);
			break;

		case 109 : // m
			particleBlue++;
			setupColor(false);
			break;

			
		case 59 : // ;
			particleRed--;
			if (particleRed < 0) { particleRed = 0; }
			setupColor(false);
			break;

		case 58 : // :
			particleGreen--;
			if (particleGreen < 0) { particleGreen = 0; }
			setupColor(false);
			break;

		case 33 : // !
			particleBlue--;
			if (particleBlue < 0) { particleBlue = 0; }
			setupColor(false);
			break;
		}
	}
}


void *updateParticles (void *arg)
{
	int id = (intptr_t) arg;
	for (int i = firstParticle[id]; i < lastParticle[id]; i++) { particles[i].update(); }
	pthread_exit(NULL);
	
}

void *moveParticles (void *arg)
{
	int id = (intptr_t) arg;
	for (int i = firstParticle[id]; i < lastParticle[id]; i++) { particles[i].move(); }
	pthread_exit(NULL);
}

void *applyParticles (void *arg)
{
	int id = (intptr_t) arg;
	for (int i = firstParticle[id]; i < lastParticle[id]; i++) { particles[i].apply(); }
	pthread_exit(NULL);
}

void *clearPixels (void *arg)
{
	int id = (intptr_t) arg;
	for (int i = firstPixel[id]; i < lastPixel[id]; i++) { pixels[i] = 0; }
	pthread_exit(NULL);
}


void *applyPixels (void *arg)
{
	int id = (intptr_t) arg;
	uchar *pixel = frame->ptr<uchar>(0);
	for (int i = firstPixel[id]; i < lastPixel[id]; i++)
	{
		int c = pixels[i];
		int i3 = i*3;
		pixel[i3] = blueColor[c];
		pixel[i3+1] = greenColor[c];
		pixel[i3+2] = redColor[c];		
	}
}



Particle::Particle ()
{
	px = py = dx = dy = 0;
}


void Particle::update ()
{
	float dist = sqrt(pow(bodyX-px,2)+pow(bodyY-py,2));
	if (dist == 0 || (gravitationRadius > 0 && dist > gravitationRadius)) { return; }
	if (gravitationRadius > 0) { dist /= gravitationRadius; }

	float addX = 0; float addY = 0;

    switch (gravitationType) {

    case SYMMETRIC_GRAVITATION :
    {
      float factor = bodyWeight / pow(dist,gravitationFactor);
	  if (gravitationRadius > 0) { factor /= gravitationRadius; }
      addX = (bodyX-px) * factor;
      addY = (bodyY-py) * factor;
    }
    break;

    case EXPONENTIAL_GRAVITATION :
    {
      float powDist = pow(dist,gravitationFactor);
      float factor = gravitationFactor * powDist / pow(dist,2);

      float dx = bodyX-px;
      float expX = exp(dx/graphicsWidth);
      float xRightFactor = bodyLeftWeight * expX;
      float xLeftFactor = bodyRightWeight / expX;
      addX = factor * dx * (xRightFactor + xLeftFactor) - (xRightFactor - xLeftFactor) / (graphicsWidth * powDist);

      float dy = bodyY-py;
      float expY = exp(dy/graphicsHeight);
      float yBottomFactor = bodyTopWeight * expY;
      float yTopFactor = bodyBottomWeight / expY;
      addY = factor * dy * (yBottomFactor + yTopFactor) - (yBottomFactor - yTopFactor) / (graphicsHeight * powDist);
    }
    break;
    }

	dx += addX; dy += addY;
}

  
void Particle::move ()
{
	dx *= (1.-particleDamping);
	dy *= (1.-particleDamping);

	px += dx*gravitationSpeed;
	py += dy*gravitationSpeed;
	
	while (px < 0 || px >= graphicsWidth) {
		if (px < 0) { px = -px; } else if (px >= graphicsWidth) { px = 2*graphicsWidth-px-1; }
		dx = -dx;
	}
		
	while (py < 0 || py >= graphicsHeight) {
		if (py < 0) { py = -py; } else if (py >= graphicsHeight) { py = 2*graphicsHeight-py-1; }
		dy = -dy;
	}
}


void Particle::apply ()
{
	pixels[((int)px) + ((int)py) * graphicsWidth]++;
}


int ms_sleep (unsigned int ms)
{
	int result = 0;

	{
		struct timespec ts_remaining =
			{ 
				ms / 1000, 
				(ms % 1000) * 1000000L 
			};

		do
		{
			struct timespec ts_sleep = ts_remaining;
			result = nanosleep(&ts_sleep, &ts_remaining);
		} 
		while (EINTR == result);
	}

	if (result)
	{
		perror("nanosleep() failed");
		result = -1;
	}

	return result;
}


