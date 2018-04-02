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

#include "static_cells.hpp"


// CONFIGURATION

const int graphicsWidth = 2032;
const int graphicsHeight = 1016;

const int gravitationType = SYMMETRIC_GRAVITATION;
const int borderType = MIRROR_BORDER;
const int initType = UNIFORM_INIT;

const float initialBodyAttractPower = 4;
const float initialBodyRepelPower = 4;
const float initialGravitationFactor = 1;
const float initialGravitationRadius = 0;
const float initialGravitationSpeed = 1;

const int initialParticleNumber = 300000;
const float initialGravitationDamping = 0.02;

const int initialColor = 0;
const float initialParticleIntensity = 4;

const int initialThreadNumber = 6;

const int colorNb = 4;
const int particleRedArray[]   = {3, 12,  6, 7};
const int particleGreenArray[] = {6,  6,  5, 7};
const int particleBlueArray[]  = {12, 3, 12, 7};


// VARIABLES

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
float gravitationDamping = initialGravitationDamping;
float bodyAttractPower = initialBodyAttractPower;
float bodyRepelPower = initialBodyRepelPower;

bool connectedBodies = true;
bool verbose = false;
int graphicsFps = 0;


// GRAPHICS VARIABLES

#define MILLION 1000000L;

bool stop = false;
const int pixelNumber = graphicsWidth * graphicsHeight;
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

float bodyX, bodyY, bodyWeight;
float bodyLeftWeight, bodyRightWeight, bodyTopWeight, bodyBottomWeight;


// SCREEN VARIABLES

double rate = 5;
std::default_random_engine generator;
std::exponential_distribution<double> distribution (1./rate);

double cfmin = 0.5;
double sfmax = 0.5;

ScreenVector screens;
int screenNb;
int delayMax = 1;


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
	setup();
	while (!stop)
	{
		getTime();
		if (connectedBodies) { computeBodies(); }
		computeParticles();
		draw();
	}

	return 0;
}


Body::Body () {
	closestBody = 0;
	minDist = -1;
}


void mouseEvents (int event, int x, int y, int flags, void *userdata)
{
	if (event == cv::EVENT_LBUTTONDOWN)
		bodyWeight = bodyAttractPower;

	else if (event == cv::EVENT_LBUTTONUP)
		bodyWeight = 0;

	if (event == cv::EVENT_RBUTTONDOWN)
		bodyWeight = - bodyRepelPower;

	else if (event == cv::EVENT_RBUTTONUP)
		bodyWeight = 0;

	else if (event == cv::EVENT_MOUSEMOVE)
	{
		bodyX = x;
		bodyY = y;
	}
}


void setup ()
{
	srand(time(0));
	stop = false;
	setupScreens();

	// SETUP PARTICLES
	particles = new Particle [initialParticleNumber];
	initParticles(initType);

	redColor = new int [initialParticleNumber+1];
	blueColor = new int [initialParticleNumber+1];
	greenColor = new int [initialParticleNumber+1];

	setupColor(true);
	
	// SETUP DISPLAY
	cv::namedWindow("static-cells", CV_WINDOW_NORMAL);
	cv::setWindowProperty ("static-cells", CV_WND_PROP_FULLSCREEN, 1);
	cv::setMouseCallback ("static-cells", mouseEvents, NULL);
		  
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


void setupScreens ()
{
	screens.push_back (Screen ('A',    5,   0, 704, 144));
	screens.push_back (Screen ('B',  987,   0, 256, 416));
	screens.push_back (Screen ('C', 1351,   0, 448, 144));
	screens.push_back (Screen ('D', 1808,   0, 224, 144));
	screens.push_back (Screen ('E',  264, 160, 448, 256));
	screens.push_back (Screen ('F', 1447, 160, 352, 256));
	screens.push_back (Screen ('G',    0, 366, 255, 288));
	screens.push_back (Screen ('H',  354, 429, 256, 224));
	screens.push_back (Screen ('I', 1363, 429, 256, 224));
	screens.push_back (Screen ('J', 1808, 360, 224, 656));
	screens.push_back (Screen ('K',  170, 664, 896, 352));
	screens.push_back (Screen ('L', 1265, 664, 448, 352));

	screenNb = screens.size();
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
		ss << "Damping: " << gravitationDamping;
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

	for (int s = 0; s < screenNb; s++)
		cv::rectangle (*frame, screens[s].srectp, cv::Scalar (0, 0, 255));

	cv::imshow ("static-cells", *frame);
	
	int key = cv::waitKey (1);
	if (key > 0)
	{
		key = key & 0xFF;
		std::cout << "KEY PRESSED: " << key << std::endl;
		stop = stop || key == 27; // ESC

		if (key == 13) // Enter
		{
			time_t t = time(0);
			struct tm *now = localtime (&t);
			std::string filename = "static-cells-screenshot-" + std::to_string (now->tm_year + 1900) + "-" + std::to_string (now->tm_mon + 1) + "-" + std::to_string (now->tm_mday) + "-" + std::to_string (now->tm_hour) + "-" + std::to_string (now->tm_min) + "-" + std::to_string (now->tm_sec) + ".png";

			std::vector<int> params;
			params.push_back (CV_IMWRITE_PNG_COMPRESSION);
			params.push_back (0);
			cv::imwrite (filename, *frame, params);
			std::cout << "SCREENSHOT: " << filename << std::endl;
		}

		if (key == 48) saveConfig (0);
		if (key == 49) saveConfig (1);
		if (key == 50) saveConfig (2);
		if (key == 51) saveConfig (3);
		if (key == 52) saveConfig (4);
		if (key == 53) saveConfig (5);
		if (key == 54) saveConfig (6);
		if (key == 55) saveConfig (7);
		if (key == 56) saveConfig (8);
		if (key == 57) saveConfig (9);

		if (key == 224) loadConfig (0);
		if (key ==  38) loadConfig (1);
		if (key == 233) loadConfig (2);
		if (key ==  34) loadConfig (3);
		if (key ==  39) loadConfig (4);
		if (key ==  40) loadConfig (5);
		if (key ==  45) loadConfig (6);
		if (key == 232) loadConfig (7);
		if (key ==  95) loadConfig (8);
		if (key == 231) loadConfig (9);

		if (key == 85) // Page up
		{ connectedBodies = true; }
		if (key == 86) // Page down
		{ connectedBodies = false; }
		
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
			gravitationDamping -= 0.002;
			if (gravitationDamping < 0) { gravitationDamping = 0; }
			break;

		case 181 : // 5 (vernum)
			gravitationDamping = initialGravitationDamping;
			break;

		case 182 : // 6 (vernum)
			gravitationDamping += 0.002;
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
	dx *= (1.-gravitationDamping);
	dy *= (1.-gravitationDamping);

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



void saveConfig (int index) {
	std::stringstream ss;
	ss << "static-cells-config-" << index;
	std::string filename = ss.str();

	std::ofstream file;
	file.open (filename, std::ios::out | std::ios::trunc);

	file << "particleNumber " << particleNumber << "\n";
	file << "particleIntensity " << particleIntensity << "\n";
	file << "particleRed " << particleRed << "\n";
	file << "particleGreen " << particleGreen << "\n";
	file << "particleBlue " << particleBlue << "\n";
	file << "bodyAttractPower " << bodyAttractPower << "\n";
	file << "bodyRepelPower " << bodyRepelPower << "\n";
	file << "gravitationRadius " << gravitationRadius << "\n";
	file << "gravitationFactor " << gravitationFactor << "\n";
	file << "gravitationSpeed " << gravitationSpeed << "\n";
	file << "gravitationDamping " << gravitationDamping << "\n";
	file << "threadNumber " << threadNumber << "\n";

	file.close ();

	std::cout << "CONFIG FILE " << index << " SAVED" << std::endl;
}


void loadConfig (int index) {
	std::stringstream ss;
	ss << "static-cells-config-" << index;
	std::string filename = ss.str();

	std::ifstream file;
	file.open (filename, std::ios::in);

	std::string var;
	std::string val;
	
	while (file >> var >> val) {
		if (var == "particleNumber") particleNumber = atoi (val.c_str());
		else if (var == "particleIntensity") particleIntensity = atof (val.c_str());
		else if (var == "particleRed") particleRed = atoi (val.c_str());
		else if (var == "particleGreen") particleGreen = atoi (val.c_str());
		else if (var == "particleBlue") particleBlue = atoi (val.c_str());
		else if (var == "bodyAttractPower") bodyAttractPower = atof (val.c_str());
		else if (var == "bodyRepelPower") bodyRepelPower = atof (val.c_str());
		else if (var == "gravitationRadius") gravitationRadius = atof (val.c_str());
		else if (var == "gravitationFactor") gravitationFactor = atof (val.c_str());
		else if (var == "gravitationSpeed") gravitationSpeed = atof (val.c_str());
		else if (var == "gravitationDamping") gravitationDamping = atof (val.c_str());
		else if (var == "threadNumber") threadNumber = atoi (val.c_str());
	}

	file.close ();

	setupColor(false);
	setupThreads();

	std::cout << "CONFIG FILE " << index << " LOADED" << std::endl;
}



Screen::Screen (char vi, int vx, int vy, int vdx, int vdy, int vdelay) {
	i = vi; active = false; delay = vdelay;
	sx = vx; sy = vy; sdx = vdx; sdy = vdy;
	srect = cv::Rect (sx, sy, sdx, sdy);
	srectp = cv::Rect (sx-1, sy-1, sdx+2, sdy+2);
	time = distribution (generator);
}

void Screen::set ()
{
	double cfmax = std::min (graphicsHeight * sfmax / sdx, graphicsWidth * sfmax / sdy);
	double f = ((double) rand() / (RAND_MAX)) * (cfmax - cfmin) + cfmin;

	int vdx = f * sdx;
	int vdy = f * sdy;
	int vx = (rand() % (graphicsWidth-2 - vdx)) + 1;
	int vy = (rand() % (graphicsHeight-2 - vdy)) + 1;

	int vdelay = rand() % delayMax;

	set (vx, vy, vdx, vdy, vdelay);
}

void Screen::set (int vx, int vy, int vdx, int vdy, int vdelay)
{
	active = true; delay = vdelay;
	cx = vx; cy = vy; cdx = vdx; cdy = vdy;
	crect = cv::Rect (cx, cy, cdx, cdy);
	crectp = cv::Rect (cx-1, cy-1, cdx+2, cdy+2);
	time = distribution (generator);
	std::cout << "SCREEN " << i << " ON: " << cx << " " << cy << " " << cdx << " " << cdy << std::endl;
}

void Screen::unset ()
{
	active = false; time = distribution (generator);
	std::cout << "SCREEN " << i << " OFF" << std::endl;
}
	
void Screen::swap () {
	if (active) unset ();
	else set ();
}
