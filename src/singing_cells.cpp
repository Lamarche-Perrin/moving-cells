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


#include <iostream>
#include <iomanip>
#include <thread>
#include <vector>
#include <cstring>
#include <cmath>

#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alut.h>
#include <sndfile.h>

#include <opencv2/opencv.hpp>

#include "singing_cells.hpp"

#define PI 3.14159265

cv::Mat3b frame (1000, 1000, cv::Vec3b (0, 0, 0));

bool mouseMove = false;
int mouseX, mouseY;

ALfloat sampleDuration = 3;
ALsizei sampleRate = 44100;
ALenum sampleFormat = AL_FORMAT_MONO16;

ALfloat sampleStep = 1. / sampleRate;
ALsizei sampleNumber = sampleRate * sampleDuration;
ALfloat currentTime = 0;

double windowWidth = 1000;
double windowHeight = 500;

ALfloat windowSampleDuration = 0.01;
ALsizei windowSampleNumber = sampleRate * windowSampleDuration;

std::vector<ALshort> currentSamples;
cv::Mat plot;

int main () {
	// Parameters
	double frequency1 = 440;
	double amplitude1 = 1000;
	double frequency2 = 4;
	double amplitude2 = 100;

	// Create device and context
	ALCdevice *device = alcOpenDevice (NULL);
	if (! device) { std::cerr << "Could not create OpenAL device" << std::endl; return EXIT_FAILURE; } else { std::cout << "Creating OpenAL device" << std::endl; }

	ALCcontext *context = alcCreateContext (device, NULL);
	if (! context) { std::cerr << "Could not create OpenAL context" << std::endl; return EXIT_FAILURE; } else { std::cout << "Creating OpenAL context" << std::endl; }
	if (! alcMakeContextCurrent (context)) { std::cerr << "Could not activate OpenAL context" << std::endl; return EXIT_FAILURE; } else { std::cout << "Activating OpenAL context" << std::endl; }

	// Create window
	cv::namedWindow ("singing-cells", CV_WINDOW_AUTOSIZE);
	cv::setMouseCallback ("singing-cells", mouseEvents, NULL);

	cv::namedWindow ("singing-plot", CV_WINDOW_NORMAL);
	cv::resizeWindow ("singing-plot", windowWidth, windowHeight);

	currentSamples = std::vector<ALshort> (sampleNumber);
	sinWave (currentSamples, amplitude2, frequency2);
	amWave (currentSamples, amplitude1, frequency1);

	plotWave (currentSamples);
	plotParameters (amplitude1, frequency1, amplitude2, frequency2);

	std::thread windowThread (displayFrame);

	// Create samples
	bool stop = false;
	while (! stop) {
		if (mouseMove) {
			frequency1 = mouseX;
			frequency2 = mouseY / 10;
			mouseMove = false;
		} else { continue; }
		
		sinWave (currentSamples, amplitude2, frequency2);
		amWave (currentSamples, amplitude1, frequency1);
		plotWave (currentSamples);
		plotParameters (amplitude1, frequency1, amplitude2, frequency2);
		
		// Create buffer
		ALuint buffer;
		alGenBuffers (1, &buffer);
		alBufferData (buffer, sampleFormat, &currentSamples[0], sampleNumber * sizeof (ALushort), sampleRate);
		if (alGetError() != AL_NO_ERROR) { std::cerr << "Problem while feeding samples" << std::endl; return EXIT_FAILURE; } //else { std::cout << "Feeding samples" << std::endl; }

		// Create source
		ALuint source;
		alGenSources (1, &source);
		alSourcei (source, AL_LOOPING, AL_TRUE);
		alSourcei (source, AL_BUFFER, buffer);

		// Playing samples
		alSourcePlay (source);
		ALint status;
		do
		{
			// ALfloat seconds = 0.f;
			// alGetSourcef (source, AL_SEC_OFFSET, &seconds);
			//std::cout << "\rPlaying samples... " << std::fixed << std::setprecision(2) << seconds << " sec";
			if (mouseMove) { alSourceStop (source); }
			alGetSourcei (source, AL_SOURCE_STATE, &status);
		} while (status == AL_PLAYING);

		// Delete buffer and source
		alSourcei (source, AL_BUFFER, 0);
		alDeleteBuffers (1, &buffer);
		alDeleteSources (1, &source);
	}
	
	// Delete context and device
    alcMakeContextCurrent (NULL);
    alcDestroyContext (context);
    alcCloseDevice (device);

	windowThread.join();
	cv::destroyAllWindows();
	
	return EXIT_SUCCESS;
}


void mouseEvents (int event, int x, int y, int flags, void *userdata)
{
	
	if (event == cv::EVENT_MOUSEMOVE) {
		mouseMove = true;
		mouseX = x;
		mouseY = y;
		//std::cout << "MOUSE CLICK (" << x << "," << y << ")" << std::endl;
	}
}

void displayFrame () {
	while (true) {
		cv::imshow ("singing-cells", frame);
		cv::imshow ("singing-plot", plot);
		cv::waitKey(1);
	}
}



void sinWave (std::vector<ALshort> &samples, float amplitude, float frequency)
{
	ALfloat time = currentTime;
	for (unsigned int i = 0; i < sampleNumber; i++) {
		samples[i] = amplitude * cos (2 * PI * frequency * time);
		time += sampleStep;
	}
}


void addWave (std::vector<ALshort> &samples, const std::vector<ALshort> &samplesToAdd)
{
	for (unsigned int i = 0; i < sampleNumber; i++) { samples[i] += samplesToAdd[i]; }
}


void amWave (std::vector<ALshort> &samples, double amplitude, double frequency)
{
	ALfloat time = currentTime;
	for (unsigned int i = 0; i < sampleNumber; i++) {
		samples[i] *= amplitude * cos (2 * PI * frequency * time);
		time += sampleStep;
	}
}


void fmWave (std::vector<ALshort> &samples, double amplitude, double frequency, double deltaFrequency) {
	if (deltaFrequency == 0) { deltaFrequency = frequency; }
	ALfloat time = currentTime;
	ALshort sampleSum = 0;
	for (unsigned int i = 0; i < sampleNumber; i++) {
		sampleSum += samples[i];
		samples[i] = amplitude * cos (2 * PI * frequency * time + 2 * PI * deltaFrequency * (sampleSum / sampleStep));
		time += sampleStep;
	}
}

void plotWave (std::vector<ALshort> &samples) {
	cv::Scalar white (255, 255, 255);
	cv::Scalar black (0, 0, 0);
	plot = cv::Mat (windowHeight, windowWidth, CV_8UC3, black);

	double max = 0;
	for (unsigned int i = 0; i < windowSampleNumber; i++) { if (abs (samples[i]) > max) { max = abs (samples[i]); } }

	double x1 = 0;
	double y1 = samples[0] * windowHeight / (2 * max) + windowHeight / 2;
	double x2;
	double y2;
	
	for (unsigned int i = 0; i < windowSampleNumber-1; i++) {
		x2 = x1 + windowWidth / windowSampleNumber;
		y2 = samples[i+1] * windowHeight / (2 * max) + windowHeight / 2;
		//std::cout << x1 << " " << y1 << " " << x2 << " " << y2 << std::endl;
		cv::line (plot, cv::Point (x1, y1), cv::Point (x2, y2), white, 1);
		x1 = x2;
		y1 = y2;
	}
}

void plotParameters (double amplitude1, double frequency1, double amplitude2, double frequency2)
{
	int x = 10;
	int y = 20;
	
	std::stringstream ss;
	std::string str;

	ss.str("");
	ss << "a1 = " << amplitude1;
	str = ss.str();
	cv::putText (plot, str, cv::Point (x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
	y += 20;

	ss.str("");
	ss << "f1 = " << frequency1;
	str = ss.str();
	cv::putText (plot, str, cv::Point (x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
	y += 20;

	ss.str("");
	ss << "a2 = " << amplitude2;
	str = ss.str();
	cv::putText (plot, str, cv::Point (x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
	y += 20;

	ss.str("");
	ss << "f2 = " << frequency2;
	str = ss.str();
	cv::putText (plot, str, cv::Point (x,y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 2);
	y += 20;

}


int testOpenAL ()
{
	// Parameters
	std::string filename = "bass.wav";
	double multiplier = 2;
	
	// List OpenAL devices
    const ALCchar* deviceList = alcGetString (NULL, ALC_DEVICE_SPECIFIER);
    if (deviceList) {
		std::cout << "Listing OpenAL devices:" << std::endl;
        while (std::strlen (deviceList) > 0) {
			std::cout << "-- " << deviceList << std::endl;
			deviceList += std::strlen (deviceList) + 1;
		}
	} else { std::cerr << "Could not list OpenAL device" << std::endl; return EXIT_FAILURE; }

	// Create device and context
	ALCdevice *device = alcOpenDevice (NULL);
	if (! device) { std::cerr << "Could not create OpenAL device" << std::endl; return EXIT_FAILURE; } else { std::cout << "Creating OpenAL device" << std::endl; }

	ALCcontext *context = alcCreateContext (device, NULL);
	if (! context) { std::cerr << "Could not create OpenAL context" << std::endl; return EXIT_FAILURE; } else { std::cout << "Creating OpenAL context" << std::endl; }
	if (! alcMakeContextCurrent (context)) { std::cerr << "Could not activate OpenAL context" << std::endl; return EXIT_FAILURE; } else { std::cout << "Activating OpenAL context" << std::endl; }

	// Open audio file
    SF_INFO fileInfos;
    SNDFILE* file = sf_open (filename.c_str(), SFM_READ, &fileInfos);
    if (! file) { std::cerr << "Could not open audio file: " << filename << std::endl; return EXIT_FAILURE; } else { std::cout << "Opening audio file " << filename << std::endl; }
	
	ALsizei sampleNumber  = static_cast<ALsizei> (fileInfos.channels *fileInfos.frames);
	std::cout << "-- sample number: " << sampleNumber << std::endl;

    ALsizei sampleRate = static_cast<ALsizei> (fileInfos.samplerate);
	std::cout << "-- sample rate:   " << sampleRate << std::endl;

	ALenum sampleFormat;
    switch (fileInfos.channels)
    {
	case 1 : sampleFormat = AL_FORMAT_MONO16;   std::cout << "-- format:        MONO16" << std::endl; break;
	case 2 : sampleFormat = AL_FORMAT_STEREO16; std::cout << "-- format:        STEREO16" << std::endl; break;
	default : std::cout << "Unknown audio format" << std::endl; return EXIT_FAILURE;
    }
	
	// Read samples
	std::vector<ALshort> samples (sampleNumber);
    if (sf_read_short (file, &samples[0], sampleNumber) < sampleNumber) { std::cerr << "Problem while reading samples" << std::endl; return EXIT_FAILURE; } else { std::cout << "Reading samples" << std::endl; }
    sf_close (file);

	// Modifying samples
	for (int t = 0; t < sampleNumber; t++) { samples[t] = samples[t] * multiplier; }
	
	// Create buffer
	ALuint buffer;
    alGenBuffers (1, &buffer);
	alBufferData (buffer, sampleFormat, &samples[0], sampleNumber * sizeof (ALushort), sampleRate);
	if (alGetError() != AL_NO_ERROR) { std::cerr << "Problem while feeding samples" << std::endl; return EXIT_FAILURE; } else { std::cout << "Feeding samples" << std::endl; }

	// Create source
	ALuint source;
	alGenSources (1, &source);
	alSourcei (source, AL_BUFFER, buffer);

	// Playing samples
	alSourcePlay (source);
	ALint status;
	do
	{
		ALfloat seconds = 0.f;
		alGetSourcef (source, AL_SEC_OFFSET, &seconds);
		std::cout << "\rPlaying samples... " << std::fixed << std::setprecision(2) << seconds << " sec";
		
		alGetSourcei (source, AL_SOURCE_STATE, &status);
	} while (status == AL_PLAYING);

	// Delete all objects
	alSourcei (source, AL_BUFFER, 0);
	alDeleteBuffers (1, &buffer);
	alDeleteSources (1, &source);
    alcMakeContextCurrent (NULL);
    alcDestroyContext (context);
    alcCloseDevice (device);

	return EXIT_SUCCESS;
}


int makeSound ()
{
	// Parameters
	double duration = 5;
	double frequency1 = 440;
	double amplitude1 = 1000;
	double frequency2 = 4;
	double amplitude2 = 100;

	// Create device and context
	ALCdevice *device = alcOpenDevice (NULL);
	if (! device) { std::cerr << "Could not create OpenAL device" << std::endl; return EXIT_FAILURE; } else { std::cout << "Creating OpenAL device" << std::endl; }

	ALCcontext *context = alcCreateContext (device, NULL);
	if (! context) { std::cerr << "Could not create OpenAL context" << std::endl; return EXIT_FAILURE; } else { std::cout << "Creating OpenAL context" << std::endl; }
	if (! alcMakeContextCurrent (context)) { std::cerr << "Could not activate OpenAL context" << std::endl; return EXIT_FAILURE; } else { std::cout << "Activating OpenAL context" << std::endl; }

	// Create samples	
	std::vector<ALshort> samples (sampleNumber);
	for (int t = 0; t < sampleNumber; t++) {
		//samples[t] = amplitude1 * cos (t * frequency1 / sampleRate * 2 * PI);	
		samples[t] = amplitude1 * cos (2. * PI * frequency1 * (double) t / sampleRate + amplitude2 / frequency2 * sin (2. * PI * frequency2 * (double) t / sampleRate));
	}
	
	// Create buffer
	ALuint buffer;
    alGenBuffers (1, &buffer);
	alBufferData (buffer, sampleFormat, &samples[0], sampleNumber * sizeof (ALushort), sampleRate);
	if (alGetError() != AL_NO_ERROR) { std::cerr << "Problem while feeding samples" << std::endl; return EXIT_FAILURE; } else { std::cout << "Feeding samples" << std::endl; }

	// Create source
	ALuint source;
	alGenSources (1, &source);
	alSourcei (source, AL_BUFFER, buffer);

	// Playing samples
	alSourcePlay (source);
	ALint status;
	do
	{
		ALfloat seconds = 0.f;
		alGetSourcef (source, AL_SEC_OFFSET, &seconds);
		std::cout << "\rPlaying samples... " << std::fixed << std::setprecision(2) << seconds << " sec";
		
		alGetSourcei (source, AL_SOURCE_STATE, &status);
	} while (status == AL_PLAYING);

	// Delete all objects
	alSourcei (source, AL_BUFFER, 0);
	alDeleteBuffers (1, &buffer);
	alDeleteSources (1, &source);
    alcMakeContextCurrent (NULL);
    alcDestroyContext (context);
    alcCloseDevice (device);

	return EXIT_SUCCESS;
}


