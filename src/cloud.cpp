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
#include <sys/time.h>
#include <errno.h>
#include <pthread.h>
#include <time.h>

#include "cloud.hpp"


// FUNCTIONS

Cloud::Cloud () {}


Cloud::~Cloud ()
{
	setdown();
}


void Cloud::init ()
{
	addBody (mouseBody);
	updateBodies ();
	updatePhysics ();
	setup ();
}


void Cloud::addBody (Body *body) {
	body->id = newBodyList->size();
	newBodyList->push_back (body);
}


void Cloud::clearBodies () {
	// for (BodyList::iterator it = newBodyList->begin(); it != newBodyList->end(); ++it) {
	// 	if ((*it) != mouseBody) { delete (*it); }
	// }
	newBodyList = new BodyList ();
	addBody (mouseBody);
}


void *Cloud::run (void *arg)
{
	reinterpret_cast<Cloud*>(arg)->run();
	pthread_exit (NULL);
}


void Cloud::run ()
{
	computeFrame();
	if (displayParticles) displayFrame();

	while (!stop)
	{
		getTime();
		events.step (delay);
		if (readParameters && inputParameterFile) { readInputParameterFile (); }
		
#if VERBOSE
		std::cout << "FRAME NUMBER " << frameNb << std::endl;
#endif

		updateBodies ();
		updatePhysics ();
		computeParticles();
		computeFrame();

		if ((frameFrequency == 0 && frameLogFrequency == 0)
			|| (frameFrequency > 0 && frameNb % frameFrequency == 0)
			|| (frameLogFrequency > 0 && (int) (log (frameNb) / log (frameLogFrequency)) == log (frameNb) / log (frameLogFrequency))
			) {
			if (displayParticles) displayFrame();
			if (recordParticles) recordFrame();
		}

#if VERBOSE
		std::cout << std::endl;
#endif

		if (frameLimit > 0 && frameNb > frameLimit) stop = true;
	}
}


void Cloud::setup ()
{
	stop = false;
	setupParameters();
	
	// SETUP PARTICLES
	particles = new Particle [maxParticleNumber];
	initParticles (particleInitMode);

	particleRedArray = new int [maxParticleNumber * pixelResolution + 1];
	particleBlueArray = new int [maxParticleNumber * pixelResolution + 1];
	particleGreenArray = new int [maxParticleNumber * pixelResolution + 1];

	setupColor ();

	if (configFilename != "") {
		if (outputFilename == "") { outputFilename = "out/" + configFilename.substr (configFilename.rfind ("/") + 1); }
	} else {
		outputFilename = "out/static-cells-screenshot";
	}
	
	// SETUP DISPLAY
	if (displayParticles) {
		if (SDL_Init(SDL_INIT_EVERYTHING) != 0) { std::cerr << "Failed to initialize SDL: " << SDL_GetError() << std::endl; }
		else {
			int windowMode = 0;
			if (displayFullscreen) { windowMode = SDL_WINDOW_FULLSCREEN_DESKTOP; }
			if (SDL_CreateWindowAndRenderer (graphicsWidth, graphicsHeight, windowMode, &window, &renderer) < 0) { std::cerr << "Error creating window or renderer: " << SDL_GetError() << std::endl; SDL_Quit(); }
			else {
				texture = SDL_CreateTexture (renderer, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING, graphicsWidth, graphicsHeight);
				if (hideMouse) { SDL_ShowCursor (SDL_DISABLE); }
			}
		}
	}
		  
	frame = new cv::Mat (graphicsHeight, graphicsWidth, CV_8UC3);
	pixels = new float [graphicsWidth*graphicsHeight];

	// SETUP THREADS
	setupThreads();

	// SETUP TIME
	frameNb = 0;
	sumFrameNb = 0;
	delay = 0;
	sumDelay = 0;
	currentDelay = 0;
	gettimeofday (&startTimer, NULL);
	parameterTimer = startTimer;

	initParticles (UNIFORM_INIT);

	// SETUP EVENTS
	setupEvents ();

	if (recordParameters) {
		openOutputParameterFile (inoutParameterFilename);
		writeOutputParameterFile ();
	}
	else if (readParameters) { openInputParameterFile (inoutParameterFilename); }
}


void Cloud::setdown ()
{
	if (recordParameters) { closeOutputParameterFile(); }
	else if (readParameters) { closeInputParameterFile(); }
}


void Cloud::setupThreads ()
{
	pthread_attr_init (&attr);
	pthread_attr_setdetachstate (&attr, PTHREAD_CREATE_JOINABLE);

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


void Cloud::setupColor ()
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

	for (int number = 0; number <= particleNumber * pixelResolution; number++)
	{
		float particleRatio = (float) number / (particleNumber * pixelResolution) * pixelNumber;

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
		
			particleRedArray[number] = R;
			particleGreenArray[number] = G;
			particleBlueArray[number] = B;
		}
	}
}


void Cloud::getTime()
{
	gettimeofday (&endTimer, NULL);
	delay = (endTimer.tv_sec - startTimer.tv_sec) + (float) (endTimer.tv_usec - startTimer.tv_usec) / MILLION;
	startTimer = endTimer;

	if (framePerSecond > 0) {
		if (delay < 1./framePerSecond) {
			struct timespec waitTime = {0};
			waitTime.tv_sec = 0;
			waitTime.tv_nsec = (1./framePerSecond - delay) * BILLION;
			nanosleep (& waitTime, (struct timespec *) NULL);
		} else { std::cout << "WARNING: not able to have " << framePerSecond << "fps" << std::endl; }
	}

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

	if (constantDelay > 0) { delay = constantDelay; }
	currentDelay += delay;
}


void Cloud::initParticles (int type)
{
	switch (type)
	{
	case RANDOM_INIT :
	{
		for (int i = 0; i < particleNumber; i++)
		{
			Particle *particle = &particles[i];
			float rX = rand() % graphicsWidth;
			float rY = rand() % graphicsHeight;

			particle->x = rX / rDistance;
			particle->y = rY / rDistance;
			particle->dx = 0;
			particle->dy = 0;
		}	
	}
	break;

	case DYNAMIC_INIT :
	{
		for (int i = 0; i < particleNumber; i++)
		{
			Particle *particle = &particles[i];
			float rX = rand() % graphicsWidth;
			float rY = rand() % graphicsHeight;

			particle->x = rX / rDistance;
			particle->y = rY / rDistance;

			float speed = ((double) rand() / (RAND_MAX)) * rDistance / 10000;
			float angle = rand() % 360;
			particle->dx = speed * cos (angle);
			particle->dy = speed * sin (angle);
		}	
	}
	break;

	case UNIFORM_INIT :
	{
		int index = 0;
		float bin = sqrt ((float) (graphicsWidth * graphicsHeight) / particleNumber);
		for (float rX = 0; rX < graphicsWidth; rX += bin)
		{
			for (float rY = 0; rY < graphicsHeight; rY += bin)
			{
				Particle *particle = &particles[index++];
				
				particle->x = rX / rDistance;
				particle->y = rY / rDistance;
				particle->dx = 0;
				particle->dy = 0;
			}
		}

		for (int i = index; i < particleNumber; i++) {
			Particle *particle = &particles[i];
			float rX = rand() % graphicsWidth;
			float rY = rand() % graphicsHeight;

			particle->x = rX / rDistance;
			particle->y = rY / rDistance;
			particle->dx = 0;
			particle->dy = 0;
		}
	}
	break;
	}	
}


void Cloud::updateBodies ()
{
	if (bodyList != newBodyList) {
		for (BodyList::iterator it = bodyList->begin(); it != bodyList->end(); ++it) { if (*it != mouseBody) { delete (*it); } }
		delete bodyList;

		pthread_mutex_lock (&mutex);
		bodyList = newBodyList;
		pthread_mutex_unlock (&mutex);
	}
}


void Cloud::updatePhysics ()
{
	pixelNumber = graphicsWidth * graphicsHeight;
	rDelay = delay * timeFactor;
	rDistance = sqrt (pixelNumber);

	rPixelSize = 1 / rDistance;
	if (pixelCleaningRate > 0) {
		rPixelCleaningRate = exp (- rDelay * log(2) / pixelCleaningRate);
		rPixelDrawingRate = 1 - rPixelCleaningRate;
	} else {
		rPixelCleaningRate = 1;
		rPixelDrawingRate = 1;
	}
	
	rWidthBorder = graphicsWidth / rDistance;
	rWidthBorderDoubled = rWidthBorder * 2;
	rHeightBorder = graphicsHeight / rDistance;
	rHeightBorderDoubled = rHeightBorder * 2;
	
	rGravitationFactor = gravitationFactor / 2;
	rGravitationAngle = gravitationAngle * PI / 180;

	for (unsigned int j = 0; j < bodyList->size(); j++) {
		Body *body = bodyList->at(j);
		body->rX = body->x * rDistance;
		body->rY = body->y * rDistance;
	}

	rParticleDamping = particleDamping / particleWeight;
}


void Cloud::computeParticles ()
{
	ArgStruct **args = new ArgStruct *[threadNumber];

	// CLEAN OR CLEAR PIXELS
	if (pixelCleaningRate == 0) {
#if VERBOSE
		std::cout << "BEGIN clear pixels" << std::endl;
#endif
	
		for (int i = 0; i < threadNumber; i++)
		{
			args[i] = new ArgStruct (this, i);
			int rc = pthread_create (&threads[i], NULL, &Cloud::clearPixels, (void *) args[i]);
			if (rc) { std::cout << "Error: Unable to create thread " << rc << std::endl; exit(-1); }
		}

		for (int i = 0; i < threadNumber; i++)
		{
			int rc = pthread_join (threads[i], &status);
			if (rc) { std::cout << "Error: Unable to join thread " << rc << std::endl; exit(-1); }
			delete args[i];
		}

#if VERBOSE
		std::cout << "-> END clear pixels" << std::endl;
#endif
	}

	else {
#if VERBOSE
		std::cout << "BEGIN clean pixels" << std::endl;
#endif
	
		for (int i = 0; i < threadNumber; i++)
		{
			args[i] = new ArgStruct (this, i);
			int rc = pthread_create (&threads[i], NULL, &Cloud::cleanPixels, (void *) args[i]);
			if (rc) { std::cout << "Error: Unable to create thread " << rc << std::endl; exit(-1); }
		}

		for (int i = 0; i < threadNumber; i++)
		{
			int rc = pthread_join (threads[i], &status);
			if (rc) { std::cout << "Error: Unable to join thread " << rc << std::endl; exit(-1); }
			delete args[i];
		}
	}
	
#if VERBOSE
	std::cout << "-> END clear pixels" << std::endl;
#endif

	// MOVE PARTICLES
#if VERBOSE
	std::cout << "BEGIN move particles" << std::endl;
#endif

	for (int i = 0; i < threadNumber; i++)
	{
		args[i] = new ArgStruct (this, i);
		int rc = pthread_create (&threads[i], NULL, &Cloud::updateAndMoveParticles, (void *) args[i]);
		if (rc) { std::cout << "Error: Unable to create thread " << rc << std::endl; exit(-1); }
	}

	for (int i = 0; i < threadNumber; i++)
	{
		int rc = pthread_join (threads[i], &status);
		if (rc) { std::cout << "Error: Unable to join thread " << rc << std::endl; exit(-1); }
		delete args[i];
	}		
	
#if VERBOSE
	std::cout << "-> END move particles" << std::endl;
#endif

	// APPLY PIXELS TO FRAME
#if VERBOSE
	std::cout << "BEGIN apply pixels" << std::endl;
#endif

	for (int i = 0; i < threadNumber; i++)
	{
		args[i] = new ArgStruct (this, i);
		int rc = pthread_create (&threads[i], NULL, &Cloud::applyPixels, (void *) args[i]);
		if (rc) { std::cout << "Error: Unable to create thread " << rc << std::endl; exit(-1); }
	}

	for (int i = 0; i < threadNumber; i++)
	{
		int rc = pthread_join (threads[i], &status);
		if (rc) { std::cout << "Error: Unable to join thread " << rc << std::endl; exit(-1); }
		delete args[i];
	}

#if VERBOSE
	std::cout << "-> END apply pixels" << std::endl;
#endif

	delete [] args;
}


void Cloud::computeFrame ()
{
	finalFrame = frame->clone();

	if (displayBodies) {
		for (unsigned int j = 0; j < bodyList->size(); j++) {
			Body *body = bodyList->at(j);
			cv::circle (finalFrame, cv::Point (body->rX, body->rY), 1, cv::Scalar(250,200,100), 3);
		}
	}

	if (displayParameters)
	{
		int x = 10;
		int y = 20;
		
		std::stringstream ss;
		std::string str;
				 
		for (int parameter = 0; parameter < PARAMETER_NUMBER; parameter++) {
			ss.str("");
			ss << parameters[parameter].str << " = " << getParameter (parameter);
			str = ss.str();
			cv::putText (finalFrame, str, cv::Point(x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
			y += 20;
		}
		
		y += 10;

		ss.str("");
		if (borderMode == MIRROR_BORDERS) { ss << "[b] borders = TRUE"; } else { ss << "[b] borders = FALSE"; }
		str = ss.str();
		cv::putText (finalFrame, str, cv::Point(x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
		y += 20;

		ss.str("");
		ss << "mouse = (" << mouseX << "," << mouseY << ")";
		str = ss.str();
		cv::putText (finalFrame, str, cv::Point(x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
		y += 30;
		

		ss.str("");
		ss << "windows = " << graphicsWidth << " x " << graphicsHeight;
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


		struct timeval drawTimer;
		gettimeofday (&drawTimer, NULL);
		int seconds = floor ((drawTimer.tv_sec - parameterTimer.tv_sec) + (float) (drawTimer.tv_usec - parameterTimer.tv_usec) / MILLION);
		int hours = floor (seconds / 3600);
		seconds = seconds % 3600;
		int minutes = floor (seconds / 60);
		seconds = seconds % 60;
		
		ss.str("");
		ss << "time = " << std::setfill('0') << std::setw(2) << hours << ":" << std::setw(2) << minutes << ":" << std::setw(2) << seconds;
		str = ss.str();
		cv::putText (finalFrame, str, cv::Point(x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
		y += 20;
			
		ss.str("");
		ss << "frame = " << frameNb;
		str = ss.str();
		cv::putText (finalFrame, str, cv::Point(x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
		y += 20;

		ss.str("");
		ss << "graphics = " << graphicsFps << "fps";
		str = ss.str();
		cv::putText (finalFrame, str, cv::Point(x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
		y += 20;

	}

	if (displayCoordinates)
	{
		int x = 400;
		int y = 20;
		
		std::stringstream ss;
		std::string str;
		
		for (unsigned int j = 0; j < bodyList->size(); j++) {
			Body *body = bodyList->at(j);
				 
			ss.str("");
			ss << "(" << body->x << ", " << body->y << ") -> " << body->weight;
			str = ss.str();
			cv::putText (finalFrame, str, cv::Point(x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
			y += 20;
		}
	}
}


void Cloud::displayFrame ()
{
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
				if (mouseBody->weight != 0) { events.interrupt (new InstantaneousVariation (this, BODY_WEIGHT, 0)); }
				else { events.interrupt (new InstantaneousVariation (this, BODY_WEIGHT, 1)); }
				break;
					
			case SDL_BUTTON_RIGHT :
				if (mouseBody->weight != 0) { events.interrupt (new InstantaneousVariation (this, BODY_WEIGHT, 0)); }
				else { events.interrupt (new InstantaneousVariation (this, BODY_WEIGHT, 2)); }
				break;

			case SDL_BUTTON_MIDDLE :
				for (int p = 0; p < PARAMETER_NUMBER; p++) {
					if (keyboard[parameters[p].scancode]) { setParameter (p, parameters[p].moy); }
				}
				break;
			}
			break;
				
		case SDL_MOUSEMOTION:
			mouseX = event.motion.x;
			mouseY = event.motion.y;
			setParameter (BODY_X, ((float) mouseX) / rDistance);
			setParameter (BODY_Y, ((float) mouseY) / rDistance);
			break;

		case SDL_MOUSEWHEEL:
			for (int p = 0; p < PARAMETER_NUMBER; p++) {
				if (keyboard[parameters[p].scancode]) {
					float value = getParameter (p);

					if (event.wheel.y < 0) {
						value -= parameters[p].ssub * event.wheel.y;
						if (value < parameters[p].min) { value = parameters[p].min; }
					}
					else {
						value += parameters[p].aadd * event.wheel.y;
						if (value > parameters[p].max) { value = parameters[p].max; }
					}
					setParameter (p, value);
				}
			}
			break;

			// Keyboard events
		case SDL_KEYDOWN:
			// SDL_Log ("Physical %s key acting as %s key",
			// 		 SDL_GetScancodeName (event.key.keysym.scancode),
			// 		 SDL_GetKeyName (event.key.keysym.sym)
			// 	);

			switch (event.key.keysym.scancode)
			{
			case SDL_SCANCODE_ESCAPE:
				stop = true;
				break;

			case SDL_SCANCODE_KP_ENTER:
				displayParameters = !displayParameters;
				break;				

			case SDL_SCANCODE_RETURN : 
				initParticles (UNIFORM_INIT);
				break;
				
			case SDL_SCANCODE_BACKSPACE : 
				initParticles (RANDOM_INIT);
				break;
				
			case SDL_SCANCODE_EQUALS : 
				initParticles (DYNAMIC_INIT);
				break;
				
			case SDL_SCANCODE_B :
				if (borderMode == MIRROR_BORDERS) { borderMode = NO_BORDERS; } else { borderMode = MIRROR_BORDERS; }
				break;

				// Control intensity
			case SDL_SCANCODE_DELETE :
				if (pixelIntensity > 0) { events.interrupt (new LinearVariation (this, PIXEL_INTENSITY, 0, 5)); }
				else { events.interrupt (new LinearVariation (this, PIXEL_INTENSITY, 1, 4)); }
				break;

			case SDL_SCANCODE_RIGHTBRACKET : events.interrupt (new LinearVariation (this, PIXEL_INTENSITY, 0, 1)); break;
			case SDL_SCANCODE_BACKSLASH : events.interrupt (new LinearVariation (this, PIXEL_INTENSITY, 1, 0.1)); break;

				// Control parameter sequences
			case SDL_SCANCODE_GRAVE :
			{
				openInputParameterFile ("static-cells-input-sequence.csv");
				openOutputParameterFile ("static-cells-output-sequence.csv");
				readParameters = true;
				recordParameters = true;
			}
			break;
			
			case SDL_SCANCODE_0 : case SDL_SCANCODE_1 : case SDL_SCANCODE_2 : case SDL_SCANCODE_3 : case SDL_SCANCODE_4 : case SDL_SCANCODE_5 : case SDL_SCANCODE_6 : case SDL_SCANCODE_7 : case SDL_SCANCODE_8 : case SDL_SCANCODE_9 :
			{
				std::string filename;
				switch (event.key.keysym.scancode) {
				case SDL_SCANCODE_0 : filename = "static-cells-parameter-sequence-0.csv"; break;
				case SDL_SCANCODE_1 : filename = "static-cells-parameter-sequence-1.csv"; break;
				case SDL_SCANCODE_2 : filename = "static-cells-parameter-sequence-2.csv"; break;
				case SDL_SCANCODE_3 : filename = "static-cells-parameter-sequence-3.csv"; break;
				case SDL_SCANCODE_4 : filename = "static-cells-parameter-sequence-4.csv"; break;
				case SDL_SCANCODE_5 : filename = "static-cells-parameter-sequence-5.csv"; break;
				case SDL_SCANCODE_6 : filename = "static-cells-parameter-sequence-6.csv"; break;
				case SDL_SCANCODE_7 : filename = "static-cells-parameter-sequence-7.csv"; break;
				case SDL_SCANCODE_8 : filename = "static-cells-parameter-sequence-8.csv"; break;
				case SDL_SCANCODE_9 : filename = "static-cells-parameter-sequence-9.csv"; break;
				default : break;
				}

				if (recordParameters) {
					openOutputParameterFile (filename);
					writeOutputParameterFile ();
				}
				else if (readParameters) { openInputParameterFile (filename); }
				break;
			}

			case SDL_SCANCODE_PAGEUP :
			{
				struct timeval writeTimer;
				gettimeofday (&writeTimer, NULL);
				double currentTimestamp = (writeTimer.tv_sec - parameterTimer.tv_sec) + (float) (writeTimer.tv_usec - parameterTimer.tv_usec) / MILLION;

				std::stringstream ss;
				ss.str("");
				ss << "particle-positions-" << currentTimestamp << ".csv";
				recordParticlePositions (ss.str());
			}
			break;
			
			case SDL_SCANCODE_PAGEDOWN : readParticlePositions ("particle-positions.csv"); break;

				// Control body weight
			case SDL_SCANCODE_SPACE :
				if (mouseBody->weight != 0) { events.interrupt (new InstantaneousVariation (this, BODY_WEIGHT, 0)); }
				else { events.interrupt (new InstantaneousVariation (this, BODY_WEIGHT, 0.5)); }
				break;

			case SDL_SCANCODE_LEFT :
				events.interrupt (new InstantaneousVariation (this, BODY_WEIGHT, 0.75));
				break;

			case SDL_SCANCODE_UP :
				events.interrupt (new InstantaneousVariation (this, BODY_WEIGHT, 1.));
				break;

			case SDL_SCANCODE_DOWN :
				events.interrupt (new InstantaneousVariation (this, BODY_WEIGHT, 1.));
				break;

			case SDL_SCANCODE_RIGHT :
				events.interrupt (new InstantaneousVariation (this, BODY_WEIGHT, 1.5));
				break;

				// Control all parameters
			case SDL_SCANCODE_KP_PERIOD :
				for (int p = 0; p < PARAMETER_NUMBER; p++) {
					if (keyboard[parameters[p].scancode]) {
						setParameter (p, parameters[p].min);
					}
				}
				break;

			case SDL_SCANCODE_KP_0 :
				for (int p = 0; p < PARAMETER_NUMBER; p++) {
					if (keyboard[parameters[p].scancode]) {
						setParameter (p, parameters[p].moy);
					}
				}
				break;

			case SDL_SCANCODE_KP_PLUS : case SDL_SCANCODE_KP_9 : case SDL_SCANCODE_KP_MULTIPLY : case SDL_SCANCODE_KP_MINUS :
				for (int p = 0; p < PARAMETER_NUMBER; p++) {
					if (keyboard[parameters[p].scancode]) {
						float value = getParameter (p);
							
						switch (event.key.keysym.scancode)
						{
						case SDL_SCANCODE_KP_PLUS :     value += parameters[p].aadd; break;
						case SDL_SCANCODE_KP_9 :        value += parameters[p].add; break;
						case SDL_SCANCODE_KP_MULTIPLY : value += parameters[p].sub; break;
						case SDL_SCANCODE_KP_MINUS :    value += parameters[p].ssub; break;
						default : break;
						}
							
						switch (event.key.keysym.scancode)
						{
						case SDL_SCANCODE_KP_PLUS : case SDL_SCANCODE_KP_9 :         if (value > parameters[p].max) { value = parameters[p].max; } break;
						case SDL_SCANCODE_KP_MULTIPLY : case SDL_SCANCODE_KP_MINUS : if (value < parameters[p].min) { value = parameters[p].min; } break;
						default : break;
						}
							
						setParameter (p, value);
					}
				}
				break;

				default : break;
			}
			break;
				
			// Other events
		case SDL_QUIT:
			stop = true;
			break;
		}
	}
}


void Cloud::recordFrame ()
{
	std::string frameStr = std::string (5 - floor(log10(frameNb)), '0') + std::to_string(frameNb);
	std::string filename = outputFilename + "-" + frameStr + ".png";
	std::vector<int> params;
	params.push_back (CV_IMWRITE_PNG_COMPRESSION);
	params.push_back (0);
	cv::imwrite (filename, finalFrame, params);
	std::cout << "SCREENSHOT: " << filename << std::endl;
}


void Cloud::recordParticlePositions (int index)
{
	std::stringstream ss;
	ss << "particle-positions-" << index << ".csv";
	recordParticlePositions (ss.str());
}


void Cloud::recordParticlePositions (std::string filename)
{
	std::ofstream outputParticleFile;
	outputParticleFile.open (filename, std::ios::out | std::ios::trunc);

	if (! outputParticleFile.is_open()) { std::cout << "COULD NOT OPEN FILE: " << filename << std::endl; }
	else {
		std::cout << "SAVING PARTICLE POSITIONS: " << filename << std::endl;

		for (int i = 0; i < particleNumber; i++)
		{ outputParticleFile << particles[i].x << " " << particles[i].y << " " << particles[i].dx << " " << particles[i].dy << "\n"; }	

		outputParticleFile.close ();
	}
}


void Cloud::readParticlePositions (int index)
{
	std::stringstream ss;
	ss << "particle-positions-" << index << ".csv";
	readParticlePositions (ss.str());
}


void Cloud::readParticlePositions (std::string filename)
{
	std::ifstream inputParticleFile;
	inputParticleFile.open (filename, std::ios::in);

	if (! inputParticleFile.is_open()) { std::cout << "COULD NOT OPEN FILE: " << filename << std::endl; }
	else {
		std::cout << "LOADING PARTICLE POSITIONS: " << filename << std::endl;

		int i = 0;
		std::string line;
		float x, y, dx, dy;
	
		while (std::getline (inputParticleFile, line) && i < particleNumber) {
			std::istringstream ss (line);
			ss >> x >> y >> dx >> dy;
			particles[i].x = x;
			particles[i].y = y;
			particles[i].dx = dx;
			particles[i].dy = dy;
			i++;
		}

		inputParticleFile.close ();
	}
}


void *Cloud::updateAndMoveParticles (void *args)
{
	ArgStruct *argStruct = (ArgStruct *) args;
	reinterpret_cast<Cloud*>(argStruct->cloud)->updateAndMoveParticles (argStruct->id);
	pthread_exit (NULL);
}


void Cloud::updateAndMoveParticles (int id)
{
	for (int i = firstParticle[id]; i < lastParticle[id]; i++) {

		Particle *particle = &particles[i];
		
		// Compute motion
		float ddx = - rParticleDamping * particle->dx;
		float ddy = - rParticleDamping * particle->dy;

		for (unsigned int j = 0; j < bodyList->size(); j++) {
			Body *body = bodyList->at(j);
			if (body->weight == 0) continue;

			float distanceX = particle->x - body->x;
			float distanceY = particle->y - body->y;
	
			float factor = pow (pow (distanceX, 2) + pow (distanceY, 2), rGravitationFactor);
			if (factor == 0) continue;
			
			float angle = atan2 (distanceY, distanceX);
			ddx -= body->weight * cos (angle + rGravitationAngle) / factor;
			ddy -= body->weight * sin (angle + rGravitationAngle) / factor;
		}

		// Apply motion
		particle->dx += ddx * rDelay;
		particle->dy += ddy * rDelay;

		if (particle->dx > maxParticleSpeed) { particle->dx = maxParticleSpeed; }
		if (particle->dx < -maxParticleSpeed) { particle->dx = -maxParticleSpeed; }
		if (particle->dy > maxParticleSpeed) { particle->dy = maxParticleSpeed; }
		if (particle->dy < -maxParticleSpeed) { particle->dy = -maxParticleSpeed; }
		
		particle->x += (particle->dx - ddx * rDelay / 2) * rDelay;
		particle->y += (particle->dy - ddy * rDelay / 2) * rDelay;

		if (borderMode == MIRROR_BORDERS) {
			while (particle->x < 0 || particle->x >= rWidthBorder) {
				if (particle->x < 0) { particle->x = - particle->x + rPixelSize; } else { particle->x = rWidthBorderDoubled - particle->x - rPixelSize; }
				particle->dx = -particle->dx;
			}

			while (particle->y < 0 || particle->y >= rHeightBorder) {
				if (particle->y < 0) { particle->y = - particle->y + rPixelSize; } else { particle->y = rHeightBorderDoubled - particle->y - rPixelSize; }
				particle->dy = -particle->dy;
			}

			int rX = particle->x * rDistance;
			int rY = particle->y * rDistance;
			pixels [rX + rY * graphicsWidth] += rPixelDrawingRate;
		}

		else {
			int rX = particle->x * rDistance;
			int rY = particle->y * rDistance;
			if (rX >= 0 && rX < graphicsWidth && rY >= 0 && rY < graphicsHeight) { pixels [rX + rY * graphicsWidth] += rPixelDrawingRate; }
		}

	}
	pthread_exit (NULL);
}


void *Cloud::clearPixels (void *args)
{
	ArgStruct *argStruct = (ArgStruct *) args;
	reinterpret_cast<Cloud*>(argStruct->cloud)->clearPixels (argStruct->id);
	pthread_exit (NULL);
}

void Cloud::clearPixels (int id)
{
	for (int i = firstPixel[id]; i < lastPixel[id]; i++) { pixels[i] = 0; }
}


void *Cloud::cleanPixels (void *args)
{
	ArgStruct *argStruct = (ArgStruct *) args;
	reinterpret_cast<Cloud*>(argStruct->cloud)->cleanPixels (argStruct->id);
	pthread_exit (NULL);
}

void Cloud::cleanPixels (int id)
{
	for (int i = firstPixel[id]; i < lastPixel[id]; i++) { pixels[i] = pixels[i] * rPixelCleaningRate; }
}





void *Cloud::applyPixels (void *args)
{
	ArgStruct *argStruct = (ArgStruct *) args;
	reinterpret_cast<Cloud*>(argStruct->cloud)->applyPixels (argStruct->id);
	pthread_exit (NULL);
}

void Cloud::applyPixels (int id)
{
	uchar *pixel = frame->ptr<uchar>(0);
	for (int i = firstPixel[id]; i < lastPixel[id]; i++)
	{
		int c = pixels[i] * pixelResolution;
		int i3 = i*3;
		pixel[i3] = particleBlueArray[c] * pixelIntensity;
		pixel[i3+1] = particleGreenArray[c] * pixelIntensity;
		pixel[i3+2] = particleRedArray[c] * pixelIntensity;		
	}
}



int ms_sleep (unsigned int ms)
{
	int result = 0;
	struct timespec ts_remaining = {ms / 1000, (ms % 1000) * MILLION};

	do {
		struct timespec ts_sleep = ts_remaining;
		result = nanosleep (&ts_sleep, &ts_remaining);
	} while (EINTR == result);

	if (result) { perror ("nanosleep() failed"); result = -1; }
	return result;
}




// PARAMETERS

void Cloud::openOutputParameterFile (std::string filename) {
	if (outputParameterFile.is_open()) { closeOutputParameterFile(); }
	gettimeofday (&parameterTimer, NULL);
	outputParameterFile.open (filename, std::ios::out | std::ios::trunc);
	if (outputParameterFile) { std::cout << "WRITE PARAMETER SEQUENCE: " << filename << std::endl; } else { std::cerr << "CANNOT WRITE PARAMETER SEQUENCE: " << filename << std::endl; }	
}

void Cloud::writeOutputParameterFile ()
{
	for (int p = 0; p < PARAMETER_NUMBER; p++) { setParameter (p, getParameter (p)); }
}

void Cloud::closeOutputParameterFile () { outputParameterFile.close(); }


void Cloud::openInputParameterFile (std::string filename) {
	if (inputParameterFile.is_open()) { closeInputParameterFile(); }
	gettimeofday (&parameterTimer, NULL);
	inputParameterFile.open (filename, std::ios::in);

	if (inputParameterFile) { std::cout << "READ PARAMETER SEQUENCE: " << filename << std::endl; } else { std::cerr << "CANNOT READ PARAMETER SEQUENCE: " << filename << std::endl; }	
	std::getline (inputParameterFile, inputParameterLine);
}

void Cloud::readInputParameterFile ()
{
				double currentTimestamp;
				if (constantDelay > 0) { currentTimestamp = currentDelay; }
				else {
				struct timeval readTimer;
				gettimeofday (&readTimer, NULL);
				currentTimestamp = (readTimer.tv_sec - parameterTimer.tv_sec) + (float) (readTimer.tv_usec - parameterTimer.tv_usec) / MILLION;
			}
				
	std::istringstream iss;
	double timestamp;
	std::string name;
	float value;

	if (! inputParameterFile) return;
	iss = std::istringstream (inputParameterLine);
	iss >> timestamp >> name >> value;

	while (timestamp <= currentTimestamp) {
		if (name == "particlePositions") {
				std::stringstream ss;
				ss.str("");
				ss << "particle-positions-" << value << ".csv";
				readParticlePositions (ss.str());
			}
		
		else if (name == "particleInit") { initParticles (UNIFORM_INIT); }
		
		else if (name == "borderMode") {
				if (value == 0) { borderMode = NO_BORDERS; } else { borderMode = MIRROR_BORDERS; }
			}
		
		else {
				setParameter (getParameterId (name), value, false);
				if (name == "bodyX") {
				mouseX = value * rDistance;
				SDL_WarpMouseGlobal (mouseX, mouseY);
				SDL_PumpEvents();
				SDL_FlushEvent (SDL_MOUSEMOTION);
			}
				else if (name == "bodyY") {
				mouseY = value * rDistance;
				SDL_WarpMouseGlobal (mouseX, mouseY);
				SDL_PumpEvents();
				SDL_FlushEvent (SDL_MOUSEMOTION);
			}
			}
		
		std::getline (inputParameterFile, inputParameterLine);

		if (! inputParameterFile) break;		
		iss = std::istringstream (inputParameterLine);
		iss >> timestamp >> name >> value;
	}
}

void Cloud::closeInputParameterFile () { inputParameterFile.close(); }


void Cloud::setupParameters ()
{
	Parameter p;
	parameters.clear();
	parameters.resize (PARAMETER_NUMBER);
	
	p.id       = PARTICLE_DAMPING;
	p.name     = "particleDamping";
	p.str      = "[d] particle damping";
	p.scancode = SDL_SCANCODE_D;
	p.keycode  = SDLK_d;
	p.max      = FLT_MAX;
	p.aadd     = 0.1;
	p.add      = 0.02;
	p.sub      = -0.02;
	p.ssub     = -0.1;
	p.min      = 0;
	parameters[p.id] = p;

	p.id       = GRAVITATION_FACTOR;
	p.name     = "gravitationFactor";
	p.str      = "[f] gravitation factor";
	p.scancode = SDL_SCANCODE_F;
	p.keycode  = SDLK_f;
	p.max      = FLT_MAX;
	p.aadd     = 0.1;
	p.add      = 0.01;
	p.sub      = -0.01;
	p.ssub     = -0.1;
	p.min      = -FLT_MAX;
	parameters[p.id] = p;

	p.id       = GRAVITATION_ANGLE;
	p.name     = "gravitationAngle";
	p.str      = "[a] gravitation angle";
	p.scancode = SDL_SCANCODE_Q;
	p.keycode  = SDLK_a;
	p.max      = FLT_MAX;
	p.aadd     = 5;
	p.add      = 1;
	p.sub      = -1;
	p.ssub     = -5;
	p.min      = -FLT_MAX;
	parameters[p.id] = p;

	p.id       = BODY_X;
	p.name     = "bodyX";
	p.str      = "[x] body X";
	p.scancode = SDL_SCANCODE_X;
	p.keycode  = SDLK_x;
	p.max      = 1;
	p.aadd     = 0.1;
	p.add      = 0.01;
	p.sub      = -0.01;
	p.ssub     = -0.1;
	p.min      = 0;
	parameters[p.id] = p;

	p.id       = BODY_Y;
	p.name     = "bodyY";
	p.str      = "[y] body Y";
	p.scancode = SDL_SCANCODE_Y;
	p.keycode  = SDLK_y;
	p.max      = 1;
	p.aadd     = 0.1;
	p.add      = 0.01;
	p.sub      = -0.01;
	p.ssub     = -0.1;
	p.min      = 0;
	parameters[p.id] = p;

	p.id       = BODY_WEIGHT;
	p.name     = "bodyWeight";
	p.str      = "[w] body weight";
	p.scancode = SDL_SCANCODE_Z;
	p.keycode  = SDLK_w;
	p.max      = FLT_MAX;
	p.aadd     = 0.05;
	p.add      = 0.01;
	p.sub      = -0.01;
	p.ssub     = -0.05;
	p.min      = -FLT_MAX;
	parameters[p.id] = p;

	p.id       = BODY_RADIUS;
	p.name     = "bodyRadius";
	p.str      = "[r] body radius";
	p.scancode = SDL_SCANCODE_R;
	p.keycode  = SDLK_r;
	p.max      = FLT_MAX;
	p.aadd     = 0.1;
	p.add      = 0.01;
	p.sub      = -0.01;
	p.ssub     = -0.1;
	p.min      = 0;
	parameters[p.id] = p;

	p.id       = PIXEL_INTENSITY;
	p.name     = "pixelIntensity";
	p.str      = "[i] pixel intensity";
	p.scancode = SDL_SCANCODE_I;
	p.keycode  = SDLK_i;
	p.max      = 1;
	p.aadd     = 0.01;
	p.add      = 0.001;
	p.sub      = -0.001;
	p.ssub     = -0.01;
	p.min      = 0;
	parameters[p.id] = p;
	
	p.id       = TIME_FACTOR;
	p.name     = "timeFactor";
	p.str      = "[t] time factor";
	p.scancode = SDL_SCANCODE_T;
	p.keycode  = SDLK_t;
	p.max      = FLT_MAX;
	p.aadd     = 0.5;
	p.add      = 0.1;
	p.sub      = -0.1;
	p.ssub     = -0.5;
	p.min      = 0;
	parameters[p.id] = p;

	for (int p = 0; p < PARAMETER_NUMBER; p++) { parameters[p].moy = getParameter (p); }
}



int Cloud::getParameterId (std::string name)
{
	for (int p = 0; p < PARAMETER_NUMBER; p++) { if (parameters[p].name == name) return p; }
	return -1;
}


float Cloud::getParameter (int parameter)
{
	switch (parameter) {
	case PARTICLE_DAMPING    : return particleDamping;
	case GRAVITATION_FACTOR  : return gravitationFactor;
	case GRAVITATION_ANGLE   : return gravitationAngle;
	case BODY_X              : return mouseBody->x;
	case BODY_Y              : return mouseBody->y;
	case BODY_WEIGHT         : return mouseBody->weight;
	case BODY_RADIUS         : return mouseBody->radius;
	case PIXEL_INTENSITY     : return pixelIntensity;
	case TIME_FACTOR         : return timeFactor;
	default                  : return 0;
	}
}


void Cloud::setParameter (int parameter, float value, bool write)
{
	switch (parameter) {
	case PARTICLE_DAMPING    : particleDamping = value;     break;
	case GRAVITATION_FACTOR  : gravitationFactor = value;   break;
	case GRAVITATION_ANGLE   : gravitationAngle = value;    break;
	case BODY_X              : mouseBody->x = value;        break;
	case BODY_Y              : mouseBody->y = value;        break;
	case BODY_WEIGHT         : mouseBody->weight = value;   break;
	case BODY_RADIUS         : mouseBody->radius = value;   break;
	case PIXEL_INTENSITY     : pixelIntensity = value;      break;
	case TIME_FACTOR         : timeFactor = value;          break;
	}

	if (write && recordParameters) {
		struct timeval recordTimer;
		gettimeofday (&recordTimer, NULL);
		double timestamp = (recordTimer.tv_sec - parameterTimer.tv_sec) + (float) (recordTimer.tv_usec - parameterTimer.tv_usec) / MILLION;
		outputParameterFile << timestamp << " " << parameters[parameter].name << " " << value << "\n";
	}
}


void Cloud::addParameter (int parameter, float value, bool write) { setParameter (parameter, getParameter (parameter) + value, write); }



// EVENT CLASS


void Cloud::setupEvents ()
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


Event::Event (Cloud *vCloud, int vParameter, float vDuration) {
	cloud = vCloud;
	parameter = vParameter;
	duration = vDuration;
}


Event::~Event () {}


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


InstantaneousVariation::InstantaneousVariation (Cloud *vCloud, int vParameter, float vEndValue) : Event (vCloud, vParameter), endValue (vEndValue) {}

void InstantaneousVariation::start ()
{
	std::cout << "START INSTANTANEOUS VARIATION ON " << cloud->parameters[parameter].name << std::endl;
	Event::start ();
}

float InstantaneousVariation::step (float delay)
{
	stopped = true;
	cloud->setParameter (parameter, endValue);

	Event::step (0);
	return (delay);
}

void InstantaneousVariation::stop ()
{
	Event::stop ();
	std::cout << "STOP INSTANTANEOUS VARIATION ON " << cloud->parameters[parameter].name << std::endl;
}


LinearVariation::LinearVariation (Cloud *vCloud, int vParameter, float vEndValue, float vDuration) : Event (vCloud, vParameter, vDuration), endValue (vEndValue) {}

void LinearVariation::start ()
{
	std::cout << "START LINEAR VARIATION ON " << cloud->parameters[parameter].name << std::endl;
	Event::start ();
	startValue = cloud->getParameter (parameter);
}

float LinearVariation::step (float delay)
{
	float stepDelay = delay;
	if (currentTime + stepDelay > startTime + duration) {
		stepDelay = startTime + duration - currentTime;
		stopped = true;
	}
	
	float stepValue = (currentTime + stepDelay - startTime) / duration * (endValue - startValue) + startValue;
	cloud->setParameter (parameter, stepValue);

	Event::step (stepDelay);
	return (delay - stepDelay);
}

void LinearVariation::stop ()
{
	Event::stop ();
	std::cout << "STOP LINEAR VARIATION ON " << cloud->parameters[parameter].name << std::endl;
}


SinusoidalVariation::SinusoidalVariation (Cloud *vCloud, int vParameter, float vAmplitude, float vFrequency, float vDuration) : Event (vCloud, vParameter, vDuration), amplitude (vAmplitude), frequency (vFrequency) {}

void SinusoidalVariation::start ()
{
	std::cout << "START SINUSOIDAL VARIATION ON " << cloud->parameters[parameter].name << std::endl;
	Event::start();
	startValue = cloud->getParameter (parameter);
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
	cloud->setParameter (parameter, stepValue);

	Event::step (stepDelay);
	return (delay - stepDelay);
}

void SinusoidalVariation::stop ()
{
	Event::stop ();
	std::cout << "STOP SINUSOIDAL VARIATION ON " << cloud->parameters[parameter].name << std::endl;
}



SawtoothVariation::SawtoothVariation (Cloud *vCloud, int vParameter, float vEndValue, float vFrequency, float vDuration) : Event (vCloud, vParameter, vDuration), endValue (vEndValue), frequency (vFrequency) {}

void SawtoothVariation::start ()
{
	std::cout << "START SAWTOOTH VARIATION ON " << cloud->parameters[parameter].name << std::endl;
	Event::start();
	startValue = cloud->getParameter (parameter);
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
	cloud->setParameter (parameter, stepValue);

	Event::step (stepDelay);
	return (delay - stepDelay);
}

void SawtoothVariation::stop ()
{
	Event::stop ();
	std::cout << "STOP SAWTOOTH VARIATION ON " << cloud->parameters[parameter].name << std::endl;
}



// COLOR FUNCTIONS

RgbColor HsvToRgb (HsvColor hsv)
{
    RgbColor rgb;
    unsigned char region, remainder, p, q, t;

    if (hsv.s == 0)
    {
        rgb.r = hsv.v;
        rgb.g = hsv.v;
        rgb.b = hsv.v;
        return rgb;
    }

    region = hsv.h / 43;
    remainder = (hsv.h - (region * 43)) * 6; 

    p = (hsv.v * (255 - hsv.s)) >> 8;
    q = (hsv.v * (255 - ((hsv.s * remainder) >> 8))) >> 8;
    t = (hsv.v * (255 - ((hsv.s * (255 - remainder)) >> 8))) >> 8;

    switch (region)
    {
	case 0:
		rgb.r = hsv.v; rgb.g = t; rgb.b = p;
		break;
	case 1:
		rgb.r = q; rgb.g = hsv.v; rgb.b = p;
		break;
	case 2:
		rgb.r = p; rgb.g = hsv.v; rgb.b = t;
		break;
	case 3:
		rgb.r = p; rgb.g = q; rgb.b = hsv.v;
		break;
	case 4:
		rgb.r = t; rgb.g = p; rgb.b = hsv.v;
		break;
	default:
		rgb.r = hsv.v; rgb.g = p; rgb.b = q;
		break;
    }

    return rgb;
}

HsvColor RgbToHsv (RgbColor rgb)
{
    HsvColor hsv;
    unsigned char rgbMin, rgbMax;

    rgbMin = rgb.r < rgb.g ? (rgb.r < rgb.b ? rgb.r : rgb.b) : (rgb.g < rgb.b ? rgb.g : rgb.b);
    rgbMax = rgb.r > rgb.g ? (rgb.r > rgb.b ? rgb.r : rgb.b) : (rgb.g > rgb.b ? rgb.g : rgb.b);

    hsv.v = rgbMax;
    if (hsv.v == 0)
    {
        hsv.h = 0;
        hsv.s = 0;
        return hsv;
    }

    hsv.s = 255 * long(rgbMax - rgbMin) / hsv.v;
    if (hsv.s == 0)
    {
        hsv.h = 0;
        return hsv;
    }

    if (rgbMax == rgb.r)
        hsv.h = 0 + 43 * (rgb.g - rgb.b) / (rgbMax - rgbMin);
    else if (rgbMax == rgb.g)
        hsv.h = 85 + 43 * (rgb.b - rgb.r) / (rgbMax - rgbMin);
    else
        hsv.h = 171 + 43 * (rgb.r - rgb.g) / (rgbMax - rgbMin);

    return hsv;
}

