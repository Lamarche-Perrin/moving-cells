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
#include <algorithm>
#include <sys/time.h>

#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>


// PARAMETERS

const int initialWidth = 512;
const int initialHeight = 424;
const int scaleFactor = 1.8;

const int width = initialWidth * scaleFactor; //1024;
const int height = initialHeight * scaleFactor; //768;

const int ghostNb = 3;
const int ghostDelay[] = {5,10,15};

// VARIABLES

#define MILLION 1000000L;

bool stopKinect;

int currentIndex;
cv::Mat *currentFrame;
uchar *currentPixel;

cv::Mat **matArray;
libfreenect2::Frame **rgbArray;
libfreenect2::Frame **depthArray;
libfreenect2::Frame **bigdepthArray;
libfreenect2::Frame **undistortedArray;
libfreenect2::Frame **registeredArray;
libfreenect2::FrameMap *framesArray;

double timer;
struct timeval startTimer, endTimer;

	
// FUNCTIONS

void fillDepthMat (float *pixel);
void addNeighbour (float *pixel, int i, float &sum, int &nb);

void sigint_handler(int s) { stopKinect = true; }

int main(int argc, char *argv[])
{
  std::string program_path(argv[0]);
  size_t executable_name_idx = program_path.rfind("time-ghosts");

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

  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth);
  libfreenect2::FrameMap frames;

  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);
  dev->start();

  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

  libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());

  cv::namedWindow ("time-ghosts", cv::WINDOW_NORMAL);
  cv::setWindowProperty ("time-ghosts", cv::WND_PROP_FULLSCREEN, 1);

  int frameNb = 0;
  stopKinect = false;

  int delay = 1;
  for (int i = 0; i < ghostNb; i++) { delay += ghostDelay[i]; }
	
  matArray = new cv::Mat* [delay];
  rgbArray = new libfreenect2::Frame* [delay];
  depthArray = new libfreenect2::Frame* [delay];
  bigdepthArray = new libfreenect2::Frame* [delay];
  undistortedArray = new libfreenect2::Frame* [delay];
  registeredArray = new libfreenect2::Frame* [delay];
  framesArray = new libfreenect2::FrameMap [delay];

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
		
      if (frameNb >= delay)
	{
	  delete matArray[currentIndex];
	  delete undistortedArray[currentIndex];
	  delete registeredArray[currentIndex];
	  delete bigdepthArray[currentIndex];
	  //listener.release(framesArray[currentIndex]);			
	}

      listener.release(framesArray[currentIndex]);			
      listener.waitForNewFrame(framesArray[currentIndex]);

      rgbArray[currentIndex] = (framesArray[currentIndex])[libfreenect2::Frame::Color];
      depthArray[currentIndex] = (framesArray[currentIndex])[libfreenect2::Frame::Depth];

      cv::Mat currentDepthMap = cv::Mat (depthArray[currentIndex]->height, depthArray[currentIndex]->width, CV_32FC1, depthArray[currentIndex]->data);
      //float *CDPixel = currentDepthMap.ptr<float>(0);
      //fillDepthMat (CDPixel);

      /*
	cv::Mat temp, temp2, small_depthf;
	cv::Mat depthf (initialHeight, initialWidth, CV_8UC1);
	currentDepthMap.convertTo (depthf, CV_8UC1, 255.0/4500.0);
	cv::resize(depthf, small_depthf, cv::Size(), 0.2, 0.2);
	cv::inpaint(small_depthf, (small_depthf == 0), temp, 5.0, cv::INPAINT_TELEA);
	resize(temp, temp2, depthf.size());
	temp2.copyTo(depthf, (depthf == 0));
	depthf.convertTo (currentDepthMap, CV_32FC1, 4500.0/255.0);
      */
		
      undistortedArray[currentIndex] = new libfreenect2::Frame (initialWidth, initialHeight, 4);
      registeredArray[currentIndex] = new libfreenect2::Frame (initialWidth, initialHeight, 4);

      bigdepthArray[currentIndex] = new libfreenect2::Frame (1920, 1082, 4);
		
      registration->apply(rgbArray[currentIndex], depthArray[currentIndex], undistortedArray[currentIndex], registeredArray[currentIndex], true, bigdepthArray[currentIndex]);

      //cv::Mat mat = cv::Mat (bigdepthArray[currentIndex]->height, bigdepthArray[currentIndex]->width, CV_32FC1, bigdepthArray[currentIndex]->data);

      //float *pixel = mat.ptr<float>(0);

      //for (int i = 0; i < 1920*1080; i++) { if (pixel[i] > 5000) { pixel[i] = 0; } }

      //float max = 0; float min = 10000;
      //for (int i = 0; i < 1920*1080; i++) { if (pixel[i] > max) { max = pixel[i]; } if (pixel[i] < min && pixel[i] != 0) { min = pixel[i]; } }
      //std::cout << min << " " << max << std::endl;

      /*
	cv::Mat temp, temp2, small_depthf;
	cv::Mat depthf (bigdepthArray[currentIndex]->height, bigdepthArray[currentIndex]->width, CV_8UC1);
	mat.convertTo (depthf, CV_8UC1, 255.0/4500.0);
	cv::resize(depthf, small_depthf, cv::Size(), 0.2, 0.2);
	cv::inpaint(small_depthf, (small_depthf == 0), temp, 1.0, cv::INPAINT_TELEA);
	cv::resize(temp, temp2, depthf.size());
	temp2.copyTo(depthf, (depthf == 0));
	depthf.convertTo (mat, CV_32FC1, 4500.0/255.0);
      */
      //cv::imshow ("mat", mat/4500.0f);

		
      if (frameNb > delay)
	{
	  cv::Mat CDMat = cv::Mat (bigdepthArray[currentIndex]->height, bigdepthArray[currentIndex]->width, CV_32FC1, bigdepthArray[currentIndex]->data).clone();
	  cv::Mat CRMat = cv::Mat (rgbArray[currentIndex]->height, rgbArray[currentIndex]->width, CV_8UC4, rgbArray[currentIndex]->data).clone();
				
	  float *CDPixel = CDMat.ptr<float>(0);
	  uchar *CRPixel = CRMat.ptr<uchar>(0);

	  int previousIndex = currentIndex;
			
	  for (int i = 0; i < ghostNb; i++)
	    {
	      previousIndex -= ghostDelay[i];
	      if (previousIndex < 0) { previousIndex += delay; }
	      //std::cout << previousIndex << " ";
				
	      cv::Mat PDMat = cv::Mat (bigdepthArray[previousIndex]->height, bigdepthArray[previousIndex]->width, CV_32FC1, bigdepthArray[previousIndex]->data);
	      cv::Mat PRMat = cv::Mat (rgbArray[previousIndex]->height, rgbArray[previousIndex]->width, CV_8UC4, rgbArray[previousIndex]->data);
			
	      float *PDPixel = PDMat.ptr<float>(0);
	      uchar *PRPixel = PRMat.ptr<uchar>(0);

	      for (int i = 0; i < 1920*1080; i++)
		{
		  if (PDPixel[i+1920] < CDPixel[i+1920] - 30)
		    {
		      CDPixel[i] = PDPixel[i];
		      CRPixel[i*4] = PRPixel[i*4];
		      CRPixel[i*4+1] = PRPixel[i*4+1];
		      CRPixel[i*4+2] = PRPixel[i*4+2];
		      CRPixel[i*4+3] = PRPixel[i*4+3];
		    }
		}
	    }
			
	  //cv::flip(PRMat, CRMat, 1);
	  cv::GaussianBlur (CRMat, CRMat, cv::Size(5,5), 3);

	  float realWidth = 1080.0 * 512.0 / 424.0;
	  cv::Mat cropped (CRMat, cv::Rect ((int) ((1920.0 - realWidth)/2), 0, (int)(realWidth), 1080));

	  // float displayWidth = 720.0 * realWidth / 1080.0;
	  // cv::resize (cropped, cropped, cv::Size((int)(displayWidth),720));
	  cv::imshow ("time-ghosts", cropped);
	}

      // 512*424
      // 1920*1080
      // 1280*720
		
      int key = cv::waitKey(1);
      if (key > 0) { std::cout << "KINECT KEY PRESSED: " << key << std::endl; }
      stopKinect = stopKinect || (key > 0 && ((key & 0xFF) == 27));

      frameNb++;
    }

  dev->stop();
  dev->close();

  delete registration;

  return 0;
}


void addNeighbour (float *pixel, int i, float &sum, int &nb)
{
  if (i >= 0 && i < initialWidth*initialHeight)
    {
      float nv = pixel[i];
      if (nv != 0) { sum += nv; nb++; }
      //if (nv != 0)
      //	if (sum == 0) { sum = nv; nb = 1; }
      //	else { sum = std::min(nv,sum); nb = 1; }
    }
}


void fillDepthMat (float *pixel)
{
  bool filled = false;
  while (!filled)
    {
      filled = true;
      for (int i = 0; i < initialWidth*initialHeight; i++)
	{
	  if (pixel[i] == 0)
	    {
	      float sum = 0; int nb = 0;

	      addNeighbour(pixel, i-1-initialWidth, sum, nb);
	      addNeighbour(pixel, i-initialWidth, sum, nb);
	      addNeighbour(pixel, i+1-initialWidth, sum, nb);
	      addNeighbour(pixel, i-1, sum, nb);
	      addNeighbour(pixel, i+1, sum, nb);
	      addNeighbour(pixel, i-1+initialWidth, sum, nb);
	      addNeighbour(pixel, i+initialWidth, sum, nb);
	      addNeighbour(pixel, i+1+initialWidth, sum, nb);

	      if (nb > 0) { pixel[i] = sum/nb; }
	      else { filled = false; }
	    }
	}				
    }
}
