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
#include <SDL.h>

#include <opencv2/opencv.hpp>

#include "static_cells.hpp"


// CONFIGURATION

// DISPLAY
#define VERBOSE 0

bool displayLive         = true;
bool displaySDL          = true;
bool displayFullscreen   = true;
bool displayParameters   = true;
bool displayMouse        = true;
bool hideMouse           = true;

bool recordingLive       = false;

float frameLogFrequency  = 2.;
int frameFrequency       = 1;
int frameLimit           = 0; //4096+1;

int graphicsWidth        = 1280; //1920; //1024; //540*2; //640;
int graphicsHeight       = 720; //1080; //768; //960; //768;
int threadNumber         = 6;

int borderMode           = NO_BORDERS;
int mouseMode            = SWITCH_ON_CLICK;
int particleInitMode     = UNIFORM_INIT;

int particleNumber       = 1280*720/4; // 259200
float particleSpeed      = 0.001;
float particleDamping    = 0.02;

float gravitationFactor  = -0.1;
float gravitationAngle   = 0.;

float bodyX              = graphicsWidth / 2;
float bodyY              = graphicsHeight / 2;
float bodyWeight         = 0.;
float bodyRadius         = 0.;
float bodyAttractFactor  = 1.;
float bodyRepelFactor    = 1.;

RgbColor particleColorMin = RgbColor (  0,   0,   0);
RgbColor particleColorMoy = RgbColor (  3,   6,  12);
RgbColor particleColorMax = RgbColor (255, 255, 255);

float particleRatioMin = 0.;
float particleRatioMoy = 0.8;
float particleRatioMax = 1000.;

float pixelIntensity = 1;

bool withScreens = false;

bool withStencil = false;
std::string stencilFilename = "../tools/stencil.test.png";

bool withDistribution = true;
std::string distributionFilename = "../tools/distribution.brain.csv";
float distributionParticleDamping = 0.05;

bool withFixedBody      = false;
float fixedBodyX        = 1./3;
float fixedBodyY        = 1./3;
float fixedBodyWeight   = 4;

bool withSymmetricBody  = false;


const int maxParticleNumber   = 600000;
const float maxParticleSpeed = 1000000;
const int maxThreadNumber     = 8;


// VARIABLES

// int mouseX, mouseY;
// bool mouseConnexion = false;

bool connectedBodies = true;
bool distributedBodies = false;
int graphicsFps = 0;


float savedGravitationDamping = particleDamping;


// GRAPHICS VARIABLES

std::string configFilename = "";
std::string outputFilename = "";

SDL_Window *window;
SDL_Renderer *renderer;
SDL_Texture *texture;
SDL_Event event;
ParameterVector parameters;

bool stop;
int pixelNumber;
float gravitationDistanceFactor;
float particleSpeedFactor;
float realBodyRadius;

struct timeval startTimer, endTimer;
int frameNb;
int sumFrameNb;
float delay;
float sumDelay = 0;

Particle *particles;
int *pixels;
cv::Mat *frame;
cv::Mat finalFrame;
int frameIndex;
int firstFrameIndex = 0;

int *particleRedArray;
int *particleGreenArray;
int *particleBlueArray;

float bodyLeftWeight, bodyRightWeight, bodyTopWeight, bodyBottomWeight;

float bodyX2, bodyY2, bodyWeight2;
float bodyLeftWeight2, bodyRightWeight2, bodyTopWeight2, bodyBottomWeight2;

EventList events;
cv::Mat stencilFrame;


// SCREEN VARIABLES

float rate = 5;
std::default_random_engine generator;
std::exponential_distribution<float> distribution (1./rate);

float cfmin = 0.5;
float sfmax = 0.5;

ScreenVector screens;
int screenNb;
int delayMax = 1;


// THREAD VARIABLES

void *status;
pthread_attr_t attr;
pthread_mutex_t mutex;

pthread_t loopThread;
pthread_t threads [maxThreadNumber];
int firstParticle [maxThreadNumber];
int lastParticle [maxThreadNumber];
int firstPixel [maxThreadNumber];
int lastPixel [maxThreadNumber];


// FUNCTIONS


int main (int argc, char *argv[])
{
	if (argc > 1) { configFilename = argv[1]; }
	if (argc > 2) { outputFilename = argv[2]; }

	setup ();
	if (displayLive) draw();

	while (!stop)
	{
		getTime();
		events.step (delay);
		
#if VERBOSE
		std::cout << "FRAME NUMBER " << frameNb << std::endl;
#endif
		
		if (connectedBodies) computeBodies();
		if (distributedBodies) computeDistributedBodies();
		computeParticles();
		draw();

		if ((frameFrequency == 0 && frameLogFrequency == 0)
			|| (frameFrequency > 0 && frameNb % frameFrequency == 0)
			|| (frameLogFrequency > 0 && (int) (log (frameNb) / log (frameLogFrequency)) == log (frameNb) / log (frameLogFrequency))
			) {
			if (displayLive) display();
			if (recordingLive) record();
		}

#if VERBOSE
		std::cout << std::endl;
#endif

		if (frameLimit > 0 && frameNb > frameLimit) stop = true;
	}

	return 0;
}


Body::Body () {
	closestBody = 0;
	minDist = -1;
}


void mouseEvents (int event, int x, int y, int flags, void *userdata)
{
	if (withFixedBody) return;
						   
	if (event == cv::EVENT_MOUSEMOVE)
	{
		bodyX = x;
		bodyY = y;
	}

	else {
		if (mouseMode == SWITCH_ON_CLICK) {
			if (event == cv::EVENT_LBUTTONDOWN)
				connectedBodies = !connectedBodies;
		}

		else if (mouseMode == KEEP_PRESSED) {
			if (event == cv::EVENT_LBUTTONDOWN) {
				connectedBodies = true;
			}

			else if (event == cv::EVENT_LBUTTONUP) {
				connectedBodies = false;
			}
			
			if (event == cv::EVENT_RBUTTONDOWN) {
				connectedBodies = true;
			}

			else if (event == cv::EVENT_RBUTTONUP) {
				connectedBodies = false;
			}
		}
	}
}



void setup ()
{
	srand (time (0));

	stop = false;
	pixelNumber = graphicsWidth * graphicsHeight;

	setupParameters();
	if (withScreens) setupScreens();
	
	// SETUP PARTICLES
	particles = new Particle [maxParticleNumber];
	initParticles (particleInitMode);

	particleRedArray = new int [maxParticleNumber+1];
	particleBlueArray = new int [maxParticleNumber+1];
	particleGreenArray = new int [maxParticleNumber+1];

	setupPhysics ();
	setupColor ();
	if (withStencil) setupStencil();
	if (withDistribution) setupDistribution();

	if (configFilename != "") {
		if (outputFilename == "") { outputFilename = "out/" + configFilename.substr (configFilename.rfind ("/") + 1); }
		loadConfig (configFilename);
	} else {
		outputFilename = "out/static-cells-screenshot";
	}

	if (withFixedBody) {
		bodyX = fixedBodyX * graphicsWidth;
		bodyY = fixedBodyY * graphicsHeight;
		bodyWeight = fixedBodyWeight;
	}
	
	// SETUP DISPLAY
	if (displayLive) {
		if (displaySDL) {
			if (SDL_Init(SDL_INIT_EVERYTHING) != 0) { std::cerr << "Failed to initialize SDL: " << SDL_GetError() << std::endl; }
			else {
				int windowMode = 0;
				if (displayFullscreen) { windowMode = SDL_WINDOW_FULLSCREEN_DESKTOP; }
				if (SDL_CreateWindowAndRenderer (graphicsWidth, graphicsHeight, windowMode, &window, &renderer) < 0) { std::cerr << "Error creating window or renderer: " << SDL_GetError() << std::endl; SDL_Quit(); }
				else {
					texture = SDL_CreateTexture (renderer, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING, graphicsWidth, graphicsHeight);
					if (hideMouse) { int mouseStatus = SDL_ShowCursor (SDL_DISABLE); }
				}
			}
		} else {
			if (displayFullscreen) {
				cv::namedWindow("static-cells", CV_WINDOW_NORMAL);
				cv::setWindowProperty ("static-cells", CV_WND_PROP_FULLSCREEN, 1);
			} else {
				cv::namedWindow("static-cells", CV_WINDOW_AUTOSIZE);
			}
		
			cv::setMouseCallback ("static-cells", mouseEvents, NULL);
		}
	}
		  
	frame = new cv::Mat (graphicsHeight, graphicsWidth, CV_8UC3);
	pixels = new int [graphicsWidth*graphicsHeight];

	// SETUP THREADS
	pthread_attr_init (&attr);
	pthread_attr_setdetachstate (&attr, PTHREAD_CREATE_JOINABLE);

	setupThreads();

	// SETUP TIME
	frameNb = 0;
	sumFrameNb = 0;
	delay = 0;
	sumDelay = 0;
	gettimeofday (&startTimer, NULL);

	initParticles (UNIFORM_INIT);

	// SETUP EVENTS
	setupEvents ();
}


void setupPhysics ()
{
	gravitationDistanceFactor = pow (sqrt (pixelNumber) / 2, gravitationFactor);
	particleSpeedFactor = particleSpeed * sqrt (pixelNumber);
	realBodyRadius = bodyRadius * sqrt (pixelNumber) / 2;
}


void setupStencil ()
{
	stencilFrame = cv::imread (stencilFilename, cv::IMREAD_COLOR);
}


void setupDistribution ()
{
	std::ifstream distributionFile (distributionFilename);

	int x, y;
	for (int i = 0; i < particleNumber; i++) {
		distributionFile >> x >> y;
		particles[i].body = cv::Point (x, y);
	}

	distributionFile.close ();
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


void setupColor ()
{
	float aR = (particleRatioMin * (particleColorMax.r - particleColorMoy.r) + particleRatioMoy * (particleColorMin.r - particleColorMax.r) + particleRatioMax * (particleColorMoy.r - particleColorMin.r)) / ((particleRatioMin - particleRatioMoy) * (particleRatioMin - particleRatioMax) * (particleRatioMoy - particleRatioMax));
	float bR = (particleColorMoy.r - particleColorMin.r) / (particleRatioMoy - particleRatioMin) - aR * (particleRatioMin + particleRatioMoy);
	float cR = particleColorMin.r - aR * pow (particleRatioMin, 2) - bR * particleRatioMin;

	float aG = (particleRatioMin * (particleColorMax.g - particleColorMoy.g) + particleRatioMoy * (particleColorMin.g - particleColorMax.g) + particleRatioMax * (particleColorMoy.g - particleColorMin.g)) / ((particleRatioMin - particleRatioMoy) * (particleRatioMin - particleRatioMax) * (particleRatioMoy - particleRatioMax));
	float bG = (particleColorMoy.g - particleColorMin.g) / (particleRatioMoy - particleRatioMin) - aG * (particleRatioMin + particleRatioMoy);
	float cG = particleColorMin.g - aG * pow (particleRatioMin, 2) - bG * particleRatioMin;

	float aB = (particleRatioMin * (particleColorMax.b - particleColorMoy.b) + particleRatioMoy * (particleColorMin.b - particleColorMax.b) + particleRatioMax * (particleColorMoy.b - particleColorMin.b)) / ((particleRatioMin - particleRatioMoy) * (particleRatioMin - particleRatioMax) * (particleRatioMoy - particleRatioMax));
	float bB = (particleColorMoy.b - particleColorMin.b) / (particleRatioMoy - particleRatioMin) - aB * (particleRatioMin + particleRatioMoy);
	float cB = particleColorMin.b - aB * pow (particleRatioMin, 2) - bB * particleRatioMin;

	// HsvColor hsvMin = RgbToHsv (particleColorMin);
	// HsvColor hsvMoy = RgbToHsv (particleColorMoy);
	// HsvColor hsvMax = RgbToHsv (particleColorMax);

	// float aH = (particleRatioMin * (hsvMax.h - hsvMoy.h) + particleRatioMoy * (hsvMin.h - hsvMax.h) + particleRatioMax * (hsvMoy.h - hsvMin.h)) / ((particleRatioMin - particleRatioMoy) * (particleRatioMin - particleRatioMax) * (particleRatioMoy - particleRatioMax));
	// float bH = (hsvMoy.h - hsvMin.h) / (particleRatioMoy - particleRatioMin) - aH * (particleRatioMin + particleRatioMoy);
	// float cH = hsvMin.h - aH * pow (particleRatioMin, 2) - bH * particleRatioMin;

	// float aS = (particleRatioMin * (hsvMax.s - hsvMoy.s) + particleRatioMoy * (hsvMin.s - hsvMax.s) + particleRatioMax * (hsvMoy.s - hsvMin.s)) / ((particleRatioMin - particleRatioMoy) * (particleRatioMin - particleRatioMax) * (particleRatioMoy - particleRatioMax));
	// float bS = (hsvMoy.s - hsvMin.s) / (particleRatioMoy - particleRatioMin) - aS * (particleRatioMin + particleRatioMoy);
	// float cS = hsvMin.s - aS * pow (particleRatioMin, 2) - bS * particleRatioMin;

	// float aV = (particleRatioMin * (hsvMax.v - hsvMoy.v) + particleRatioMoy * (hsvMin.v - hsvMax.v) + particleRatioMax * (hsvMoy.v - hsvMin.v)) / ((particleRatioMin - particleRatioMoy) * (particleRatioMin - particleRatioMax) * (particleRatioMoy - particleRatioMax));
	// float bV = (hsvMoy.v - hsvMin.v) / (particleRatioMoy - particleRatioMin) - aV * (particleRatioMin + particleRatioMoy);
	// float cV = hsvMin.v - aV * pow (particleRatioMin, 2) - bV * particleRatioMin;

	for (int number = 0; number <= particleNumber; number++)
	{
		float particleRatio = (float) number / particleNumber * pixelNumber;

		if (particleRatio <= particleRatioMin) {
			particleRedArray[number] = particleColorMin.r;
			particleGreenArray[number] = particleColorMin.g;
			particleBlueArray[number] = particleColorMin.b;
		}
		
		else if (particleRatio >= particleRatioMax) {
			particleRedArray[number] = particleColorMax.r;
			particleGreenArray[number] = particleColorMax.g;
			particleBlueArray[number] = particleColorMax.b;
		}

		else {
			float R = aR * pow (particleRatio, 2) + bR * particleRatio + cR;
			float G = aG * pow (particleRatio, 2) + bG * particleRatio + cG;
			float B = aB * pow (particleRatio, 2) + bB * particleRatio + cB;

			if (R < 0) { R = 0; } if (R > 255) { R = 255; }
			if (G < 0) { G = 0; } if (G > 255) { G = 255; }
			if (B < 0) { B = 0; } if (B > 255) { B = 255; }
		
			// float H = aH * pow (particleRatio, 2) + bH * particleRatio + cH;
			// float S = aS * pow (particleRatio, 2) + bS * particleRatio + cS;
			// float V = aV * pow (particleRatio, 2) + bV * particleRatio + cV;

			// HsvColor HSV = HsvColor (H, S, V);
			// RgbColor RGB = HsvToRgb (HSV);

			particleRedArray[number] = R;
			particleGreenArray[number] = G;
			particleBlueArray[number] = B;

			// std::cout << number << " " << R << " " << G << " " << B << std::endl;
		}
	}
}


void getTime()
{
	gettimeofday (&endTimer, NULL);
	delay = (endTimer.tv_sec - startTimer.tv_sec) + (float) (endTimer.tv_usec - startTimer.tv_usec) / MILLION;
	startTimer = endTimer;

	frameNb++;
	sumFrameNb++;
	sumDelay += delay;

	if (sumDelay >= 3)
	{
		graphicsFps = (int) (((float) sumFrameNb) / sumDelay);
		std::cout << "GRAPHICS: " << graphicsFps << "fps" << std::endl;
		sumDelay = 0;
		sumFrameNb = 0;
	}
}


void initParticles (int type)
{
	switch (type)
	{
	case RANDOM_INIT :
	{
		for (int i = 0; i < particleNumber; i++)
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

		for (int i = index; i < particleNumber; i++) {
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
#if VERBOSE
	std::cout << "BEGIN compute bodies" << std::endl;
#endif

	if (withSymmetricBody) {
		bodyWeight2 = bodyWeight;
		bodyX2 = graphicsWidth - (bodyX+1);
		bodyY2 = graphicsHeight - (bodyY+1);
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

#if VERBOSE
	std::cout << "-> END compute bodies" << std::endl;
#endif
}


void computeDistributedBodies ()
{
#if VERBOSE
	std::cout << "BEGIN compute distributed bodies" << std::endl;
#endif

	for (int i = 0; i < threadNumber; i++)
	{
		int rc = pthread_create(&threads[i], NULL, updateParticlesWithDistribution, (void *) (intptr_t) i);
		if (rc) { std::cout << "Error:unable to create thread," << rc << std::endl; exit(-1); }
	}

#if VERBOSE
	std::cout << "-> END compute distributed bodies" << std::endl;
#endif
}


void computeParticles ()
{
	// MOVE PARTICLES
#if VERBOSE
	std::cout << "BEGIN move particles" << std::endl;
#endif

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

#if VERBOSE
	std::cout << "-> END move particles" << std::endl;
#endif

	// CLEAR PIXELS
#if VERBOSE
	std::cout << "BEGIN clear pixels" << std::endl;
#endif

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

#if VERBOSE
	std::cout << "-> END clear pixels" << std::endl;
#endif

	// APPLY PARTICLES TO PIXELS
#if VERBOSE
	std::cout << "BEGIN apply particles" << std::endl;
#endif

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

#if VERBOSE
	std::cout << "-> END apply particles" << std::endl;
#endif

	// CLEAR PIXELS TO FRAME
#if VERBOSE
	std::cout << "BEGIN apply pixels" << std::endl;
#endif

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

#if VERBOSE
	std::cout << "-> END apply pixels" << std::endl;
#endif
}


void draw ()
{
	if (withScreens) {
		finalFrame = cv::Mat (graphicsHeight, graphicsWidth, CV_8UC3, cv::Scalar(0,0,0));
		for (int s = 0; s < screenNb; s++) {
			cv::Mat truncated (*frame, screens[s].srect);
			truncated.copyTo (finalFrame (screens[s].srect));
		}
	} else { finalFrame = frame->clone(); }
	
	if (withStencil) {
		finalFrame = finalFrame.mul (stencilFrame, 1./128);
	}

	if (displayMouse) {
		cv::circle (finalFrame, cv::Point(bodyX,bodyY), 1, cv::Scalar(250,200,100), 3);
	}
		
	if (displayParameters)
	{
		int x = 10;
		int y = 20;
		
		std::stringstream ss;
		std::string str;
				 
		for (int parameter = 0; parameter < PARAMETER_NUMBER; parameter++) {
			ss.str("");
			ss << parameters[parameter].name << " = " << getParameterValue (parameter);
			str = ss.str();
			cv::putText (finalFrame, str, cv::Point(x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
			y += 20;
		}
		
		y += 10;

		ss.str("");
		if (borderMode == MIRROR_BORDERS) { ss << "[b] borders = TRUE"; } else { ss << "borders = FALSE"; }
		str = ss.str();
		cv::putText (finalFrame, str, cv::Point(x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
		y += 20;

		ss.str("");
		ss << "particles = " << particleNumber;
		str = ss.str();
		cv::putText (finalFrame, str, cv::Point(x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
		y += 20;

		ss.str("");
		ss << "threads = " << threadNumber;
		str = ss.str();
		cv::putText (finalFrame, str, cv::Point(x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
		y += 20;

		ss.str("");
		ss << "graphics = " << graphicsFps << "fps";
		str = ss.str();
		cv::putText (finalFrame, str, cv::Point(x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
		y += 20;

		ss.str("");
		ss << "frame = " << frameNb;
		str = ss.str();
		cv::putText (finalFrame, str, cv::Point(x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
		y += 20;

	}

	if (withScreens)
		for (int s = 0; s < screenNb; s++)
			cv::rectangle (finalFrame, screens[s].srectp, cv::Scalar (0, 0, 255));
}


void display ()
{
	if (displaySDL) {
		unsigned char *texture_data = NULL;
		int texture_pitch = 0;

		SDL_LockTexture (texture, 0, (void **) &texture_data, &texture_pitch);
		memcpy (texture_data, (void *) finalFrame.data, finalFrame.cols * finalFrame.rows * finalFrame.channels());
		SDL_UnlockTexture (texture);
	
		SDL_RenderClear (renderer);
		SDL_RenderCopy (renderer, texture, NULL, NULL);
		SDL_RenderPresent (renderer);

		const Uint8 *keyboard = SDL_GetKeyboardState(NULL);
		while (SDL_PollEvent (&event))
		{
            switch (event.type)
            {
				// Mouse events
			case SDL_MOUSEBUTTONDOWN :
				switch (event.button.button)
				{
				case SDL_BUTTON_LEFT :
					switch (mouseMode)
					{
					case KEEP_PRESSED : connectedBodies = true; break;
					case SWITCH_ON_CLICK : connectedBodies = !connectedBodies; break;
					}
					break;
					
				case SDL_BUTTON_RIGHT :
					break;

				case SDL_BUTTON_MIDDLE :
					for (int p = 0; p < PARAMETER_NUMBER; p++) {
						if (keyboard[parameters[p].scancode]) { setParameterValue (p, parameters[p].moy); }
					}
					break;
				}
				break;
				
			case SDL_MOUSEBUTTONUP:
				switch (event.button.button)
				{
					switch (mouseMode)
					{
					case KEEP_PRESSED : connectedBodies = false; break;
					case SWITCH_ON_CLICK : break;
					}
					break;
					
				case SDL_BUTTON_RIGHT:
					break;
				}
				break;

			case SDL_MOUSEMOTION:
				bodyX = event.motion.x;
				bodyY = event.motion.y;
				break;

			case SDL_MOUSEWHEEL:
				for (int p = 0; p < PARAMETER_NUMBER; p++) {
					if (keyboard[parameters[p].scancode]) {
						float value = getParameterValue (p);

						if (event.wheel.y < 0) {
							value -= parameters[p].ssub * event.wheel.y;
							if (value < parameters[p].min) { value = parameters[p].min; }
						}
						else {
							value += parameters[p].aadd * event.wheel.y;
							if (value > parameters[p].max) { value = parameters[p].max; }
						}
						setParameterValue (p, value);
					}
				}
				break;

				// Keyboard events
			case SDL_KEYDOWN:
				// SDL_Log ("Physical %s key acting as %s key",
				// 		 SDL_GetScancodeName (event.key.keysym.scancode),
				// 		 SDL_GetKeyName (event.key.keysym.sym)
				// 	);

				switch (event.key.keysym.sym)
				{
				case SDLK_ESCAPE:
					stop = true;
					break;

				case SDLK_KP_ENTER:
					displayParameters = !displayParameters;
					break;				

				case SDLK_RETURN : 
					initParticles (UNIFORM_INIT);
					break;
				
				case SDLK_BACKSPACE : 
					initParticles (RANDOM_INIT);
					break;
				
				case SDLK_DELETE :
					if (pixelIntensity > 0) { events.interrupt (new LinearVariation (PIXEL_INTENSITY, 0, 4)); }
					else { events.interrupt (new LinearVariation (PIXEL_INTENSITY, 1, 4)); }
					break;

				case SDLK_b :
					if (borderMode == MIRROR_BORDERS) { borderMode = NO_BORDERS; } else { borderMode = MIRROR_BORDERS; }
					break;

					// Control body weight
				case SDLK_SPACE :
					if (bodyWeight != 0) { events.interrupt (new InstantaneousVariation (BODY_WEIGHT, 0.)); }
					else { events.interrupt (new InstantaneousVariation (BODY_WEIGHT, 1.)); }
					break;

				case SDLK_LEFT :
					events.interrupt (new LinearVariation (BODY_WEIGHT, 1., 1));
					break;

				case SDLK_UP :
					events.interrupt (new InstantaneousVariation (BODY_WEIGHT, 0.));
					break;

				case SDLK_DOWN :
					events.interrupt (new LinearVariation (BODY_WEIGHT, 0., 1));
					break;

				case SDLK_RIGHT :
					events.interrupt (new LinearVariation (BODY_WEIGHT, -0.5, 1));
					break;

					// Control all parameters
				case SDLK_KP_0 :
					for (int p = 0; p < PARAMETER_NUMBER; p++) {
						if (keyboard[parameters[p].scancode]) {
							setParameterValue (p, parameters[p].moy);
						}
					}
					break;

				case SDLK_KP_PLUS : case SDLK_KP_9 : case SDLK_KP_MULTIPLY : case SDLK_KP_MINUS :
					for (int p = 0; p < PARAMETER_NUMBER; p++) {
						if (keyboard[parameters[p].scancode]) {
							float value = getParameterValue (p);
							
							switch (event.key.keysym.sym)
							{
							case SDLK_KP_PLUS :     value += parameters[p].aadd; break;
							case SDLK_KP_9 :        value += parameters[p].add; break;
							case SDLK_KP_MULTIPLY : value += parameters[p].sub; break;
							case SDLK_KP_MINUS :    value += parameters[p].ssub; break;
							}
							
							switch (event.key.keysym.sym)
							{
							case SDLK_KP_PLUS : case SDLK_KP_9 :         if (value > parameters[p].max) { value = parameters[p].max; } break;
							case SDLK_KP_MULTIPLY : case SDLK_KP_MINUS : if (value < parameters[p].min) { value = parameters[p].min; } break;
							}
							
							setParameterValue (p, value);
						}
					}
					break;
				}
				break;
				
				// Other events
			case SDL_QUIT:
				stop = true;
				break;
            }
        }
	} else {
		cv::imshow ("static-cells", finalFrame);
		
		int key = cv::waitKey (1);
		if (key > 0)
		{
			key = key & 0xFF;
			std::cout << "KEY PRESSED: " << key << std::endl;
			stop = stop || key == 27; // ESC

			if (key == 13) // Enter
			{
				time_t t = time (0);
				struct tm *now = localtime (&t);
				std::string filename = "static-cells-screenshot-" + std::to_string (now->tm_year + 1900) + "-" + std::to_string (now->tm_mon + 1) + "-" + std::to_string (now->tm_mday) + "-" + std::to_string (now->tm_hour) + "-" + std::to_string (now->tm_min) + "-" + std::to_string (now->tm_sec) + ".png";

				std::vector<int> params;
				params.push_back (CV_IMWRITE_PNG_COMPRESSION);
				params.push_back (0);
				cv::imwrite (filename, finalFrame, params);
				std::cout << "SCREENSHOT: " << filename << std::endl;
			}

			else if (key == 48) saveConfig (0);
			else if (key == 49) saveConfig (1);
			else if (key == 50) saveConfig (2);
			else if (key == 51) saveConfig (3);
			else if (key == 52) saveConfig (4);
			else if (key == 53) saveConfig (5);
			else if (key == 54) saveConfig (6);
			else if (key == 55) saveConfig (7);
			else if (key == 56) saveConfig (8);
			else if (key == 57) saveConfig (9);

			else if (key == 224) loadConfig (0);
			else if (key ==  38) loadConfig (1);
			else if (key == 233) loadConfig (2);
			else if (key ==  34) loadConfig (3);
			else if (key ==  39) loadConfig (4);
			else if (key ==  40) loadConfig (5);
			else if (key ==  45) loadConfig (6);
			else if (key == 232) loadConfig (7);
			else if (key ==  95) loadConfig (8);
			else if (key == 231) loadConfig (9);

			else if (key == 190) gravitationAngle = -30; // F1
			else if (key == 191) gravitationAngle = -25; // F2
			else if (key == 192) gravitationAngle = -20; // F3
			else if (key == 193) gravitationAngle = -15; // F4
			else if (key == 194) gravitationAngle = -10; // F5
			else if (key == 195) gravitationAngle = - 5; // F6
			else if (key == 196) gravitationAngle =   0; // F7
			else if (key == 197) gravitationAngle =   5; // F8
			else if (key == 198) gravitationAngle =  10; // F9
			else if (key == 199) gravitationAngle =  15; // F10
			else if (key == 200) gravitationAngle =  20; // F11
			else if (key == 201) gravitationAngle =  25; // F12

			else if (key == 176) // 0 vernum
			{
				if (particleDamping >= 1) {
					events.interrupt (new LinearVariation (PARTICLE_DAMPING, 0.02, 3));
				} else {
					events.interrupt (new LinearVariation (PARTICLE_DAMPING, 1, 3));
				}
			}
		
			else if (key == 85) // Page up
			{ connectedBodies = true; }
			else if (key == 86) // Page down
			{ connectedBodies = false; }
		
			else if (key == 117) { // u
				initParticles (UNIFORM_INIT);
			}
			
			else if (key == 106) { // j
				initParticles (RANDOM_INIT);
			}

			else if (key == 141) { // Enter (vernum)
				displayParameters = !displayParameters;
			}
			
			
			else if (key == 97) { // a
				bodyRadius -= 100;
				if (bodyRadius < 0) { bodyRadius = 0; }
			}

			else if (key == 101) { // e
				bodyRadius += 100;
			}

			
			
			else if (key == 113) { // q
				bodyAttractFactor -= 0.1;
				if (bodyAttractFactor < 0) { bodyAttractFactor = 0; }
			}

			else if (key == 100) { // d
				bodyAttractFactor += 0.1;
			}

			
			
			else if (key == 119) { // w
				bodyRepelFactor -= 0.1;
				if (bodyRepelFactor < 0) { bodyRepelFactor = 0; }
			}

			else if (key == 99) { // c
				bodyRepelFactor += 0.1;
			}


			
			else if (key == 114) { // r
				particleNumber -= (int) ((float) graphicsWidth * graphicsHeight / 16);
				if (particleNumber < 0) { particleNumber = 0; } 
				setupThreads();
				setupColor();
			}

			else if (key == 121) { // y
				particleNumber += (int) ((float) graphicsWidth * graphicsHeight / 16);
				if (particleNumber > maxParticleNumber) { particleNumber = maxParticleNumber; } 
				setupThreads();
				setupColor();
			}


			
			else if (key == 102) { // f
				particleRatioMoy -= 0.01;
				if (particleRatioMoy < 0) { particleRatioMoy = 0; } 
				setupColor();
			}

			else if (key == 104) { // h
				particleRatioMoy += 0.01;
				if (particleRatioMoy > 1) { particleRatioMoy = 1; } 
				setupColor();
			}

			

			else if (key == 118) { // v
				threadNumber -= 1;
				if (threadNumber < 1) { threadNumber = 1; } 
				setupThreads();
			}

			else if (key == 110) { // n
				threadNumber += 1;
				if (threadNumber > maxThreadNumber) { threadNumber = maxThreadNumber; }
				setupThreads();
			}


			
			else if (key == 183) { // 7 (vernum)
				particleSpeed -= 0.001;
				if (particleSpeed < 0) { particleSpeed = 0; }
				setupPhysics ();
			}

			else if (key == 185) { // 9 (vernum)
				particleSpeed += 0.001;
				setupPhysics ();
			}
			

			
			else if (key == 180) { // 4 (vernum)
				particleDamping -= 0.001;
				if (particleDamping < 0) { particleDamping = 0; }
			}

			else if (key == 182) { // 6 (vernum)
				particleDamping += 0.001;
			}

			
			
			else if (key == 177) { // 1 (vernum)
				gravitationFactor -= 0.1;
				setupPhysics ();
			}

			else if (key == 179) { // 3 (vernum)
				gravitationFactor += 0.1;
				setupPhysics ();
			}

			
			else if (key == 107) { // k
				particleColorMoy.r++;
				if (particleColorMoy.r > 255) { particleColorMoy.r = 255; }
				setupColor();
			}

			else if (key == 108) { // l
				particleColorMoy.g++;
				if (particleColorMoy.g > 255) { particleColorMoy.g = 255; }
				setupColor();
			}

			else if (key == 109) { // m
				particleColorMoy.b++;
				if (particleColorMoy.b > 255) { particleColorMoy.b = 255; }
				setupColor();
			}

			
			else if (key == 59) { // ;
				particleColorMoy.r--;
				if (particleColorMoy.r < 0) { particleColorMoy.r = 0; }
				setupColor();
			}

			else if (key == 58) { // :
				particleColorMoy.g--;
				if (particleColorMoy.g < 0) { particleColorMoy.g = 0; }
				setupColor();
			}

			else if (key == 33) { // !
				particleColorMoy.b--;
				if (particleColorMoy.b < 0) { particleColorMoy.b = 0; }
				setupColor();
			}
		}
	}
}


void record ()
{
	std::string frameStr = std::string (5 - floor(log10(frameNb)), '0') + std::to_string(frameNb);
	std::string filename = outputFilename + "-" + frameStr + ".png";
	std::vector<int> params;
	params.push_back (CV_IMWRITE_PNG_COMPRESSION);
	params.push_back (0);
	cv::imwrite (filename, finalFrame, params);
	std::cout << "SCREENSHOT: " << filename << std::endl;
}


void *updateParticles (void *arg)
{
	int id = (intptr_t) arg;
	for (int i = firstParticle[id]; i < lastParticle[id]; i++) { particles[i].update(); }
	pthread_exit(NULL);
}

void *updateParticlesWithDistribution (void *arg)
{
	int id = (intptr_t) arg;
	for (int i = firstParticle[id]; i < lastParticle[id]; i++) { particles[i].updateWithDistribution(); }
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
		pixel[i3] = particleBlueArray[c] * pixelIntensity;
		pixel[i3+1] = particleGreenArray[c] * pixelIntensity;
		pixel[i3+2] = particleRedArray[c] * pixelIntensity;		
	}
}


Particle::Particle ()
{
	alive = true;
	px = py = dx = dy = 0;
	body = cv::Point (0,0);
}


void Particle::update ()
{
	if (! alive) return;
	
	float distance = sqrt (pow (bodyX - px, 2) + pow (bodyY - py, 2));
	if (! (distance == 0 || (bodyRadius > 0 && distance > realBodyRadius))) {
		//if (bodyRadius > 0) { distance /= realBodyRadius; }

		float factor = bodyWeight * gravitationDistanceFactor / pow (distance, gravitationFactor);
		//if (bodyRadius > 0) { factor /= realBodyRadius; }

		// float addX, addY;

		// if (gravitationAngle == 0) {
		// 	addX = factor * (bodyX-px) / distance; 
		// 	addY = factor * (bodyY-py) / distance;
		// } else {
		float angle = atan ((bodyY-py)/(bodyX-px));
		if (bodyX < px) { angle += PI; }
		float addX = factor * cos (angle + gravitationAngle * PI / 180);
		float addY = factor * sin (angle + gravitationAngle * PI / 180);
		// }
		
		// float addX = (bodyX-px) * factor;
		// float addY = (bodyY-py) * factor;

		dx += addX; dy += addY;

		if (dx > maxParticleSpeed) { dx = maxParticleSpeed; alive = false; std::cout << "WARNING: MAX SPEED!" << std::endl;}
		if (dx < -maxParticleSpeed) { dx = -maxParticleSpeed; alive = false; std::cout << "WARNING: MAX SPEED!" << std::endl; }
		if (dy > maxParticleSpeed) { dy = maxParticleSpeed; alive = false; std::cout << "WARNING: MAX SPEED!" << std::endl; }
		if (dy < -maxParticleSpeed) { dy = -maxParticleSpeed; alive = false; std::cout << "WARNING: MAX SPEED!" << std::endl; }
	}

	if (withSymmetricBody) {
		float distance2 = sqrt (pow (bodyX2 - px, 2) + pow (bodyY2 - py, 2));
		if (! (distance2 == 0 || (bodyRadius > 0 && distance2 > realBodyRadius))) {
			//if (bodyRadius > 0) { distance2 /= realBodyRadius; }

			float factor2 = bodyWeight2 * gravitationDistanceFactor / pow (distance2, gravitationFactor);
			//if (bodyRadius > 0) { factor2 /= realBodyRadius; }
			float addX2 = (bodyX2-px) * factor2;
			float addY2 = (bodyY2-py) * factor2;

			dx += addX2; dy += addY2;
		}
	}
}


void Particle::updateWithDistribution ()
{
	if (! alive) return;
	
	float distance = sqrt (pow (body.x - px, 2) + pow (body.y - py, 2));
	float factor = bodyAttractFactor * 2 / pow (distance, gravitationFactor);
	float addX = (body.x-px) * factor;
	float addY = (body.y-py) * factor;
	
	dx += addX; dy += addY;		
}


void Particle::move ()
{
	if (! alive) return;

	dx *= (1. - particleDamping);
	dy *= (1. - particleDamping);

	px += dx * particleSpeedFactor;
	py += dy * particleSpeedFactor;

	if (borderMode == MIRROR_BORDERS) {
		int mx = 0;
		while (px < 0 || px >= graphicsWidth) {
			if (px < 0) { px = -px; } else if (px >= graphicsWidth) { px = 2*graphicsWidth-px-1; }
			dx = -dx;
			mx++;
		}

		int my = 0;
		while (py < 0 || py >= graphicsHeight) {
			if (py < 0) { py = -py; } else if (py >= graphicsHeight) { py = 2*graphicsHeight-py-1; }
			dy = -dy;
			my++;
		}
	}
}


void Particle::apply ()
{
	if (! alive) return;

	if (borderMode == MIRROR_BORDERS || (px >= 0 && px < graphicsWidth && py >= 0 && py < graphicsHeight))
	{ pixels[((int)px) + ((int)py) * graphicsWidth]++; }
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
	file << "particleSpeed " << particleSpeed << "\n";
	file << "particleDamping " << particleDamping << "\n";
	file << "gravitationFactor " << gravitationFactor << "\n";
	file << "gravitationAngle " << gravitationAngle << "\n";
	file << "bodyRadius " << bodyRadius << "\n";
	file << "bodyAttractFactor " << bodyAttractFactor << "\n";
	file << "bodyRepelFactor " << bodyRepelFactor << "\n";

	if (withFixedBody) {
		file << "withFixedBody 1\n";
		file << "fixedBodyX " << fixedBodyX << "\n";
		file << "fixedBodyY " << fixedBodyY << "\n";
		file << "fixedBodyWeight " << fixedBodyWeight << "\n";
	}
	
	file.close ();

	std::cout << "CONFIG FILE " << index << " SAVED" << std::endl;
}


void loadConfig (int index) {
	std::stringstream ss;
	ss << "static-cells-config-" << index;
	std::string filename = ss.str();

	loadConfig (filename);
}

void loadConfig (std::string filename) {
	std::ifstream file;
	file.open (filename, std::ios::in);

	std::string var;
	std::string val;
	
	while (file >> var >> val) {
		if (var == "particleNumber") particleNumber = atoi (val.c_str());
		else if (var == "particleSpeed") particleSpeed = atof (val.c_str());
		else if (var == "particleDamping") particleDamping = atof (val.c_str());
		else if (var == "gravitationFactor") gravitationFactor = atof (val.c_str());
		else if (var == "gravitationAngle") gravitationAngle = atof (val.c_str());
		else if (var == "bodyRadius") bodyRadius = atof (val.c_str());
		else if (var == "bodyAttractFactor") bodyAttractFactor = atof (val.c_str());
		else if (var == "bodyRepelFactor") bodyRepelFactor = atof (val.c_str());
		else if (var == "withFixedBody") withFixedBody = (atoi (val.c_str()) == 1);
		else if (var == "fixedBodyX") fixedBodyX = atof (val.c_str());
		else if (var == "fixedBodyY") fixedBodyY = atof (val.c_str());
		else if (var == "fixedBodyWeight") fixedBodyWeight = atof (val.c_str());
		else { std::cout << "Warning: field '" << var << "' in configuration file unknown!" << std::endl; }
	}

	file.close ();

	setupColor();
	setupPhysics();
	setupThreads();

	std::cout << "CONFIG FILE " << filename << " LOADED" << std::endl;
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
	float cfmax = std::min (graphicsHeight * sfmax / sdx, graphicsWidth * sfmax / sdy);
	float f = ((float) rand() / (RAND_MAX)) * (cfmax - cfmin) + cfmin;

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


void setupParameters ()
{
	Parameter p;

	p.id       = PARTICLE_DAMPING;
	p.name     = "[d] particle damping";
	p.scancode = SDL_SCANCODE_D;
	p.keycode  = SDLK_d;
	p.max      = 1;
	p.aadd     = 0.001;
	p.add      = 0.0001;
	p.sub      = -0.0001;
	p.ssub     = -0.001;
	p.min      = 0;
	p.physics  = false;
	parameters.push_back (p);

	p.id       = PARTICLE_SPEED;
	p.name     = "[s] particle speed";
	p.scancode = SDL_SCANCODE_S;
	p.keycode  = SDLK_s;
	p.max      = FLT_MAX;
	p.aadd     = 0.0001;
	p.add      = 0.00001;
	p.sub      = -0.00001;
	p.ssub     = -0.0001;
	p.min      = 0;
	p.physics  = true;
	parameters.push_back (p);

	p.id       = GRAVITATION_FACTOR;
	p.name     = "[f] gravitation factor";
	p.scancode = SDL_SCANCODE_F;
	p.keycode  = SDLK_f;
	p.max      = FLT_MAX;
	p.aadd     = 0.1;
	p.add      = 0.01;
	p.sub      = -0.01;
	p.ssub     = -0.1;
	p.min      = -FLT_MAX;
	p.physics  = true;
	parameters.push_back (p);

	p.id       = GRAVITATION_ANGLE;
	p.name     = "[a] gravitation angle";
	p.scancode = SDL_SCANCODE_Q;
	p.keycode  = SDLK_a;
	p.max      = FLT_MAX;
	p.aadd     = 10;
	p.add      = 1;
	p.sub      = -1;
	p.ssub     = -10;
	p.min      = -FLT_MAX;
	p.physics  = false;
	parameters.push_back (p);

	p.id       = BODY_WEIGHT;
	p.name     = "[w] body weight";
	p.scancode = SDL_SCANCODE_Z;
	p.keycode  = SDLK_w;
	p.max      = FLT_MAX;
	p.aadd     = 0.1;
	p.add      = 0.01;
	p.sub      = -0.01;
	p.ssub     = -0.1;
	p.min      = -FLT_MAX;
	p.physics  = false;
	parameters.push_back (p);

	p.id       = BODY_RADIUS;
	p.name     = "[r] body radius";
	p.scancode = SDL_SCANCODE_R;
	p.keycode  = SDLK_r;
	p.max      = FLT_MAX;
	p.aadd     = 0.1;
	p.add      = 0.01;
	p.sub      = -0.01;
	p.ssub     = -0.1;
	p.min      = 0;
	p.physics  = true;
	parameters.push_back (p);

	p.id       = BODY_ATTRACT_FACTOR;
	p.name     = "[f1] body attract factor";
	p.scancode = SDL_SCANCODE_F1;
	p.keycode  = SDLK_F1;
	p.max      = FLT_MAX;
	p.aadd     = 0.1;
	p.add      = 0.01;
	p.sub      = -0.01;
	p.ssub     = -0.1;
	p.min      = 0;
	p.physics  = false;
	parameters.push_back (p);

	p.id       = BODY_REPEL_FACTOR;
	p.name     = "[f2] body repel factor";
	p.scancode = SDL_SCANCODE_F2;
	p.keycode  = SDLK_F2;
	p.max      = FLT_MAX;
	p.aadd     = 0.1;
	p.add      = 0.01;
	p.sub      = -0.01;
	p.ssub     = -0.1;
	p.min      = 0;
	p.physics  = false;
	parameters.push_back (p);

	p.id       = PIXEL_INTENSITY;
	p.name     = "[i] pixel intensity";
	p.scancode = SDL_SCANCODE_I;
	p.keycode  = SDLK_i;
	p.max      = 1;
	p.aadd     = 0.01;
	p.add      = 0.001;
	p.sub      = -0.001;
	p.ssub     = -0.01;
	p.min      = 0;
	p.physics  = false;
	parameters.push_back (p);

	for (int p = 0; p < PARAMETER_NUMBER; p++) { parameters[p].moy = getParameterValue (p); }
}



float getParameterValue (int parameter)
{
	switch (parameter) {
	case PARTICLE_DAMPING    : return particleDamping;
	case PARTICLE_SPEED      : return particleSpeed;
	case GRAVITATION_FACTOR  : return gravitationFactor;
	case GRAVITATION_ANGLE   : return gravitationAngle;
	case BODY_WEIGHT         : return bodyWeight;
	case BODY_RADIUS         : return bodyRadius;
	case BODY_ATTRACT_FACTOR : return bodyAttractFactor;
	case BODY_REPEL_FACTOR   : return bodyRepelFactor;
	case PIXEL_INTENSITY     : return pixelIntensity;
	default                  : return 0;
	}
}


void setParameterValue (int parameter, float value)
{
	switch (parameter) {
	case PARTICLE_DAMPING    : particleDamping = value;   break;
	case PARTICLE_SPEED      : particleSpeed = value;     break;
	case GRAVITATION_FACTOR  : gravitationFactor = value; break;
	case GRAVITATION_ANGLE   : gravitationAngle = value;  break;
	case BODY_WEIGHT         : bodyWeight = value;        break;
	case BODY_RADIUS         : bodyRadius = value;        break;
	case BODY_ATTRACT_FACTOR : bodyAttractFactor = value; break;
	case BODY_REPEL_FACTOR   : bodyRepelFactor = value;   break;
	case PIXEL_INTENSITY     : pixelIntensity = value;   break;
	}
	if (parameters[parameter].physics) { setupPhysics(); }
}


void addParameterValue (int parameter, float value) { setParameterValue (parameter, getParameterValue (parameter) + value); }



// EVENT CLASS


void setupEvents ()
{
	// events.interrupt (new InstantaneousVariation (PARTICLE_SPEED, 0.001));
	// events.push_back (new SawtoothVariation (PARTICLE_SPEED, 0.004, 1.9));
}



EventList::EventList () { events.resize (PARAMETER_NUMBER); }

void EventList::clear (int parameter)
{
	for (std::list<Event*>::iterator it = events[parameter].begin(); it != events[parameter].end(); ++it) { delete *it; }
	events[parameter].clear();
}

void EventList::push_back (Event *event) { events[event->parameter].push_back (event); }
void EventList::interrupt (Event *event) {
	clear (event->parameter);
	events[event->parameter].push_back (event);
}

float EventList::step (float delay)
{
	for (int parameter = 0; parameter < PARAMETER_NUMBER; parameter++) {
		float currentDelay = delay;
		std::list<Event*>::iterator it = events[parameter].begin();
		while (it != events[parameter].end() && currentDelay > 0)
		{
			Event *event = *it;
			if (!event->hasStarted()) { event->start(); }
			currentDelay = event->step (currentDelay);
			if (event->hasStopped()) {
				event->stop();
				delete event;
				events[parameter].erase (it++);
			} else { ++it; }
		}
	}
	
	return 0;
}


Event::Event (int vParameter, float vDuration) : parameter (vParameter), duration (vDuration) {}


void Event::start () {
	started = true;
	struct timeval timer;
	gettimeofday (&timer, NULL);
	startTime = timer.tv_sec + (float) timer.tv_usec / MILLION;
	currentTime = startTime;	
}

void Event::stop () {}

bool Event::hasStarted () { return started; }
bool Event::hasStopped () { return stopped || (duration > 0 && currentTime > startTime + duration); }


float Event::step (float delay)
{
	currentTime += delay;
	return 0;
}


InstantaneousVariation::InstantaneousVariation (int vParameter, float vEndValue) : Event (vParameter), endValue (vEndValue) {}

void InstantaneousVariation::start ()
{
	std::cout << "START INSTANTANEOUS VARIATION ON " << parameters[parameter].name << std::endl;
	Event::start ();
}

float InstantaneousVariation::step (float delay)
{
	stopped = true;
	setParameterValue (parameter, endValue);

	Event::step (0);
	return (delay);
}

void InstantaneousVariation::stop ()
{
	Event::stop ();
	std::cout << "STOP INSTANTANEOUS VARIATION ON " << parameters[parameter].name << std::endl;
}


LinearVariation::LinearVariation (int vParameter, float vEndValue, float vDuration) : Event (vParameter, vDuration), endValue (vEndValue) {}

void LinearVariation::start ()
{
	std::cout << "START LINEAR VARIATION ON " << parameters[parameter].name << std::endl;
	Event::start ();
	startValue = getParameterValue (parameter);
}

float LinearVariation::step (float delay)
{
	float stepDelay = delay;
	if (currentTime + stepDelay > startTime + duration) {
		stepDelay = startTime + duration - currentTime;
		stopped = true;
	}
	
	float stepValue = (currentTime + stepDelay - startTime) / duration * (endValue - startValue) + startValue;
	setParameterValue (parameter, stepValue);

	Event::step (stepDelay);
	return (delay - stepDelay);
}

void LinearVariation::stop ()
{
	Event::stop ();
	std::cout << "STOP LINEAR VARIATION ON " << parameters[parameter].name << std::endl;
}


SinusoidalVariation::SinusoidalVariation (int vParameter, float vAmplitude, float vFrequency, float vDuration) : Event (vParameter, vDuration), amplitude (vAmplitude), frequency (vFrequency) {}

void SinusoidalVariation::start ()
{
	std::cout << "START SINUSOIDAL VARIATION ON " << parameters[parameter].name << std::endl;
	Event::start();
	startValue = getParameterValue (parameter);
	endValue = startValue + amplitude;
}

float SinusoidalVariation::step (float delay)
{
	float stepDelay = delay;
	if (duration > 0 && currentTime + stepDelay > startTime + duration) {
		stepDelay = startTime + duration - currentTime;
		stopped = true;
	}
	
	float stepValue = startValue + (endValue - startValue) * cos ((currentTime + stepDelay - startTime) / frequency * 2 * PI);
	setParameterValue (parameter, stepValue);

	Event::step (stepDelay);
	return (delay - stepDelay);
}

void SinusoidalVariation::stop ()
{
	Event::stop ();
	std::cout << "STOP SINUSOIDAL VARIATION ON " << parameters[parameter].name << std::endl;
}



SawtoothVariation::SawtoothVariation (int vParameter, float vEndValue, float vFrequency, float vDuration) : Event (vParameter, vDuration), endValue (vEndValue), frequency (vFrequency) {}

void SawtoothVariation::start ()
{
	std::cout << "START SAWTOOTH VARIATION ON " << parameters[parameter].name << std::endl;
	Event::start();
	startValue = getParameterValue (parameter);
	intermediateTime = startTime;
}

float SawtoothVariation::step (float delay)
{
	float stepDelay = delay;
	if (duration > 0 && currentTime + stepDelay > startTime + duration) {
		stepDelay = startTime + duration - currentTime;
		stopped = true;
	}

	intermediateTime += stepDelay;
	while (intermediateTime - startTime > frequency) { intermediateTime -= frequency; }

	float stepValue = startValue + (frequency - (intermediateTime - startTime)) / frequency * (endValue - startValue);
	setParameterValue (parameter, stepValue);

	Event::step (stepDelay);
	return (delay - stepDelay);
}

void SawtoothVariation::stop ()
{
	Event::stop ();
	std::cout << "STOP SAWTOOTH VARIATION ON " << parameters[parameter].name << std::endl;
}
