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


#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>


// DEFINE ENUM

#define NO_BORDER                 0
#define CYCLIC_BORDER             1
#define MIRROR_BORDER             2

#define UNIFORM_INIT              0
#define RANDOM_INIT               1

#define SYMMETRIC_GRAVITATION     0
#define QUADRANT_GRAVITATION      1
#define LINEAR_GRAVITATION        2
#define EXPONENTIAL_GRAVITATION   3


// PRE-DEFINITIONS

class Kinect;
class Object;
typedef std::list<Object*> ObjectList;


struct Pixel
{
    int x; int y; int z;
    float rx; float ry; float rz;
    Pixel (int cx, int cy, int cz) : x(cx), y(cy), z(cz), rx(0), ry(0), rz(0) {}
    Pixel (int cx, int cy, int cz, float crx, float cry, float crz) : x(cx), y(cy), z(cz), rx(crx), ry(cry), rz(crz) {}
};


class Object
{
public:
	Kinect *kinect;
    int index;
    Object *closestObject;
    double minDist;

	int pixelNb;
    int xMin, xMoy, xMax, yMin, yMoy, yMax, zMin, zMoy, zMax;
    int xMinS, xMoyS, xMaxS, yMinS, yMoyS, yMaxS, zMinS, zMoyS, zMaxS;
	bool extrema;

	int rpixelNb;
    float rxMin, rxMoy, rxMax, ryMin, ryMoy, ryMax, rzMin, rzMoy, rzMax;
	bool rextrema;

	float x, y, weight;
	
    Object (Kinect *vKinect) : kinect (vKinect), closestObject (0), minDist (-1) {}

    void getClosestObject (ObjectList *list);
    void update (double delay);
    double getDistance (Object *object);
    void print ();
};




// FUNCTIONS
class Kinect
{
public:
	const float thresholdAdd = 100;
	const int objectMinSize = 1000;
	const bool reverseXAxis = false;
	const bool reverseYAxis = true;

	const bool useSpeed = false;
	const bool realPositioning = true;
	const bool realMoy = true;

	const bool thresholdFromFile = false;

	const bool allowKinectCalibration = false;
	const bool allowSensorDisplay = true;
	const bool allowGraphicsDisplay = true;

	const int graphicsWidth = 1024; //1920;
	const int graphicsHeight = 768; //1080;
	const int distanceMax = 0;

	const int depthWidth = 512;
	const int depthHeight = 424;
	const int depthDepth = 450;

	const bool fromAbove = false;
	const float xMin = -2.30;
	const float xMax = 2.30;
	const float yMin = 0;
	const float yMax = 0;
	const float zMin = 2.00;
	const float zMax = 4.40;
	const float rMin = 0.45/1.75;
	const float rMoy = 1.35/1.75;
	const float rMax = 1.65/1.75;

	const float weightMax = 1.;
	const float weightMin = -1.;

	const int depthCropLeft = 0;
	const int depthCropRight = 0;
	const int depthCropTop = 0;
	const int depthCropBottom = 0;

	int objectCounter;
	bool stop;
	bool thresholdKinect;

	cv::Mat *thresholdFrame;
	ObjectList *objectList;
	ObjectList *newObjectList;
	//ObjectList *currentObjectList;

	bool verbose = false;
	int kinectFps = 0;

	libfreenect2::Freenect2 freenect2;
	libfreenect2::PacketPipeline *pipeline;
	libfreenect2::Freenect2Device *dev;
	libfreenect2::SyncMultiFrameListener *listener;
	libfreenect2::Registration *registration;
	libfreenect2::FrameMap frames;

	struct timeval kinectStartTimer, kinectEndTimer;
	int kinectFrameCounter;
	double kinectDelay;
	double kinectSumDelay = 0;

	void *status;
	pthread_attr_t attr;
	pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

	Kinect ();
	~Kinect ();

	static void *run (void *arg);
	void run ();

	void extractObjects (float *dPixel);
	void extractObjects (float *dPixel, libfreenect2::Frame *undepth);
	void displaySensor (cv::Mat *depthFrame);
	void calibrateKinect (int key);

	static void sigint_handler (int s);
	int scale (float z);
	float linearMap (float value, float min1, float max1, float min2, float max2);

	void loop ();
	void setup ();
	void setupThreads ();
	void setupColor (bool init);
	void setupDistribution ();
	void initParticles (int type);
	void getTime ();
	void computeObjects ();
	void computeDistributedObjects ();
	void computeParticles ();
	void draw ();
	//int ms_sleep (unsigned int ms);

	void *loop (void *arg);
	
	void *updateParticles (void *arg);
	void *updateParticlesWithDistribution (void *arg);
	void *moveParticles (void *arg);
	void *applyParticles (void *arg);

	void *clearPixels (void *arg);
	void *applyPixels (void *arg);

	void saveConfig (int index);
	void loadConfig (int index);
};

